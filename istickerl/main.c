#include "FreeRTOS.h"
#include "app_error.h"
#include "app_timer.h"
#include "ble.h"
#include "ble/ble_services_manager.h"
#include "drivers/buzzer.h"
#include "event_groups.h"
#include "hal/hal_boards.h"
#include "logic/TrackingAlgorithm.h"
#include "logic/peripherals.h"
#include "logic/serial_comm.h"
#include "logic/state_machine.h"
#include "logic/commands.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_clock.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrfx_log.h"
#include "semphr.h"
#include "task.h"
#include "version.h"

#include <stdbool.h>
#include <stdint.h>

extern xSemaphoreHandle   tx_uart_semaphore;
extern EventGroupHandle_t event_acc_sample;
extern xSemaphoreHandle   sleep_semaphore;
extern xSemaphoreHandle   command_semaphore;

extern DriverBehaviourState driver_behaviour_state;
extern IStickerErrorBits error_bits;

#if NRF_LOG_ENABLED
static TaskHandle_t m_logger_thread; // Logger thread
#endif

TaskHandle_t driver_behaviour_task_handle;

void init_tasks(void);

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name) { app_error_handler(DEAD_BEEF, line_num, p_file_name); }

/**@brief Function for the various modules initialization.
 *
 * @details Initializes various modules and services.
 */

static void modules_init(void)
{
    ret_code_t err_code;

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    NRFX_LOG_INFO("%s");

    ret_code_t err_code;

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the nrf log module.
 */
__STATIC_INLINE void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Thread for handling the logger.
 *
 * @details This thread is responsible for processing log entries if logs are deferred.
 *          Thread flushes all log entries and suspends. It is resumed by idle task hook.
 *
 * @param[in]   arg   Pointer used for passing some arbitrary information (context) from the
 *                    osThreadCreate() call to the thread.
 */
static void logger_thread(void *arg)
{
    UNUSED_PARAMETER(arg);

    while (1) {
        NRF_LOG_FLUSH();

        vTaskSuspend(NULL); // Suspend myself
    }
}

/**@brief A function which is hooked to idle task
 */
void vApplicationIdleHook(void)
{
#if NRF_LOG_ENABLED
    vTaskResume(m_logger_thread);
#endif
}

/**@brief Function for initializing the clock.
 */
static void clock_init(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
}

// Demo Threads
static TaskHandle_t m_blinky_thread;
static TaskHandle_t m_monitor_thread;

static void blinky_thread(void *arg)
{
    UNUSED_PARAMETER(arg);

    static uint32_t counter = 0;

    while (1) {
        ++counter;
        peripherals_toggle_leds();

        // ?????????????? ble_services_update_data((uint8_t *)&counter, sizeof(counter));

        vTaskDelay(2500);
    }
}

static void monitor_thread(void *arg)
{
    static uint8_t status_buffer[128];
    static uint8_t ble_buffer[16];

    static uint8_t  temperature;
    static uint8_t  bat_level;
    static uint16_t vdd;
    static float    vdd_float;

    uint32_t duration;

    UNUSED_PARAMETER(arg);

    while (1) {
        state_machine_feed_watchdog();

        temperature = peripherals_read_temperature();
        bat_level   = peripherals_read_battery_level();
        vdd         = peripherals_read_vdd();
        vdd_float   = ((float)vdd) / 1000;

        duration = timeDiff(xTaskGetTickCount(), driver_behaviour_state.last_activity_time) / 1000;
        duration = driver_behaviour_state.sleep_delay_time - duration;

        sprintf(status_buffer, "\r\n\r\nStatus: T=%dC, Bat=%d%%, Sleep=%d, VDD=%.2fV\r\n\r\n", temperature, bat_level, duration, vdd_float);
        DisplayMessage(status_buffer, 0);

        // NRFX_LOG_INFO("%s Temperature: %dC", __func__, peripherals_read_temperature());

        // uint8_t bat_level = peripherals_read_battery_level();
        // NRFX_LOG_INFO("%s Battery: %u%% VDD mV: %u", __func__, bat_level, peripherals_read_vdd());

#ifdef BLE_ENABLE
        ble_services_update_battery_level(bat_level);
#endif

        // send measurements to BLE
        memset(ble_buffer, 0x00, 16);
        memcpy( ble_buffer, (uint8_t*)(&vdd_float), 4 );
        ble_buffer[10] = temperature;
        ble_buffer[11] = bat_level;

        ble_services_update_measurement( ble_buffer, 16);

        // send status and error bit to BLE
        memset(ble_buffer, 0x00, 16);
        ble_buffer[0] = 1; // record type
        memcpy( ble_buffer+4, (uint8_t*)(&error_bits), 4 );

        ble_services_update_status( ble_buffer, 16);

        // send status and error bit to BLE
        vTaskDelay(10000);
    }
}

/**@brief Function for application main entry.
 */
int main(void)
{
    // Initialize.
    log_init();
    clock_init();

    // Start execution.
    if (pdPASS != xTaskCreate(logger_thread, "NRF_LOG", 256, NULL, 1, &m_logger_thread)) {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

    // Activate deep sleep mode.
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    modules_init();
    NRFX_LOG_INFO("Starting: %s %s (%s) FreeRTOS %s\n", DEVICE_NAME, FIRMWARE_REV, MODEL_NUM, tskKERNEL_VERSION_NUMBER);
    NRF_LOG_FLUSH();

    peripherals_init();
    state_machine_init();

    // ble_services_init();

    NRF_LOG_FLUSH();

    if (pdPASS != xTaskCreate(blinky_thread, "Blink", 64, NULL, 1, &m_blinky_thread)) {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    if (pdPASS != xTaskCreate(monitor_thread, "Monitor", 256, NULL, 1, &m_monitor_thread)) {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

    UNUSED_VARIABLE(xTaskCreate(driver_behaviour_task, "Tracking", configMINIMAL_STACK_SIZE + 200, NULL, 2, &driver_behaviour_task_handle));

    init_tasks();

    // ble_services_advertising_start();

    NRFX_LOG_INFO("%s Free Heap: %u", __func__, xPortGetFreeHeapSize());

    // Start FreeRTOS scheduler.
    vTaskStartScheduler();

    for (;;) {
        APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
    }
}

void init_tasks(void)
{
    tx_uart_semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(tx_uart_semaphore);

    sleep_semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(sleep_semaphore);

    command_semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(command_semaphore);

    event_acc_sample = xEventGroupCreate();
}