#include "FreeRTOS.h"
#include "app_error.h"
#include "app_timer.h"
#include "ble.h"
#include "ble/ble_services_manager.h"
#include "logic/peripherals.h"
#include "logic/serial_comm.h"
#include "logic/state_machine.h"
#include "drivers/buzzer.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_clock.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrfx_log.h"
#include "task.h"
#include "version.h"

#include <stdbool.h>
#include <stdint.h>

#if NRF_LOG_ENABLED
static TaskHandle_t m_logger_thread; // Logger thread
#endif

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
        buzzer_start();
        ble_services_update_data((uint8_t *)&counter, sizeof(counter));

        vTaskDelay(500);
    }
}

static void monitor_thread(void *arg)
{
    UNUSED_PARAMETER(arg);

    while (1) {
        state_machine_feed_watchdog();

        NRFX_LOG_INFO("%s Temperature: %dC", __func__, peripherals_read_temperature());

        uint8_t bat_level = peripherals_read_battery_level();
        NRFX_LOG_INFO("%s Battery: %u%% VDD mV: %u", __func__, bat_level, peripherals_read_vdd());

        ble_services_update_battery_level(bat_level);

        serial_comm_send_text();

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
    NRFX_LOG_INFO("Starting: %s %s (%s)\n", DEVICE_NAME, FIRMWARE_REV, MODEL_NUM);
    NRF_LOG_FLUSH();

    peripherals_init();
    state_machine_init();

    ble_services_init();

    NRF_LOG_FLUSH();

    if (pdPASS != xTaskCreate(blinky_thread, "Blink", 64, NULL, 1, &m_blinky_thread)) {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    if (pdPASS != xTaskCreate(monitor_thread, "Monitor", 256, NULL, 1, &m_monitor_thread)) {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

    ble_services_advertising_start();

    NRFX_LOG_INFO("%s Free Heap: %u", __func__, xPortGetFreeHeapSize());
    // Start FreeRTOS scheduler.
    vTaskStartScheduler();

    for (;;) {
        APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
    }
}