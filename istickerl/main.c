#include "FreeRTOS.h"
#include "app_error.h"
#include "app_timer.h"
#include "app_util_platform.h"
#include "ble.h"
#include "ble/ble_services_manager.h"
#include "ble/ble_task.h"
#include "drivers/buzzer.h"
#include "event_groups.h"
#include "hal/hal_boards.h"
#include "logic/commands.h"
#include "logic/configuration.h"
#include "logic/flash_data.h"
#include "logic/monitor.h"
#include "logic/peripherals.h"
#include "logic/serial_comm.h"
#include "logic/state_machine.h"
#include "logic/tracking_algorithm.h"
#include "logic/transfer_task.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_clock.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_strerror.h"
#include "nrfx_log.h"
#include "semphr.h"
#include "task.h"
#include "version.h"

#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
#include "nrf_sdm.h"
#endif

#include <stdbool.h>
#include <stdint.h>

extern xSemaphoreHandle tx_uart_semaphore;
extern xSemaphoreHandle clock_semaphore;
extern xSemaphoreHandle sleep_semaphore;
extern xSemaphoreHandle command_semaphore;
extern xSemaphoreHandle ble_command_notify_semaphore;
extern xSemaphoreHandle event_semaphore;
extern xSemaphoreHandle flash_semaphore;
extern xSemaphoreHandle terminal_buff_semaphore;
extern xSemaphoreHandle watchdog_monitor_semaphore;
extern xSemaphoreHandle flash_counter_semaphore;
extern xSemaphoreHandle acc_recording_semaphore;

extern EventGroupHandle_t event_sample_timer;
extern EventGroupHandle_t event_save_recording;
extern EventGroupHandle_t event_uart_rx;
extern EventGroupHandle_t ble_log_event;
extern EventGroupHandle_t transfer_event;
extern EventGroupHandle_t transfer_confirm_event;
extern EventGroupHandle_t event_acc_process_sample;

extern ResetData           reset_data;
extern DeviceConfiguration device_config;

static uint32_t reset_count __attribute__((section(".non_init")));
static uint32_t test_value __attribute__((section(".non_init")));

uint32_t reset_count_x;

#if NRF_LOG_ENABLED
static TaskHandle_t m_logger_thread; // Logger thread
#endif

TaskHandle_t driver_behaviour_task_handle;
TaskHandle_t transfer_task_handle;
TaskHandle_t sampler_task_handle;

extern DriverBehaviourState driver_behaviour_state;

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
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

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

#if NRF_LOG_ENABLED && NRF_LOG_DEFERRED
void log_pending_hook(void)
{
    BaseType_t result = pdFAIL;

    if (__get_IPSR() != 0) {
        BaseType_t higherPriorityTaskWoken = pdFALSE;

        // result = xTaskNotifyFromISR(m_logger_thread, 0, eSetValueWithoutOverwrite, &higherPriorityTaskWoken);
        result = xTaskResumeFromISR(m_logger_thread);

        if (pdFAIL != result) {
            portYIELD_FROM_ISR(higherPriorityTaskWoken);
        }
    } else {
        UNUSED_RETURN_VALUE(xTaskNotify(m_logger_thread, 0, eSetValueWithoutOverwrite));
    }
}
#endif

/**@brief A function which is hooked to idle task.
 * @note Idle hook must be enabled in FreeRTOS configuration (configUSE_IDLE_HOOK).
 */
void vApplicationIdleHook(void)
{
#if NRF_LOG_ENABLED
    // vTaskResume(m_logger_thread);
#endif
}

/**@brief Function for initializing the clock.
 */
static void clock_init(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}

static TaskHandle_t m_monitor_thread;
static TaskHandle_t m_ble_thread;

int main(void)
{
    // Initialize.
    log_init();
    clock_init();

    if (test_value != 0x5A5A5A5A) {
        test_value = 0x5A5A5A5A;

        reset_count = 0;
        SetTimeFromString("010120", "000000");
        // clear_calibration();
        // driver_behaviour_state.calibrated = false;
        reset_data.reason = RESET_POWER_OFF;
    } else {
        reset_count++;

        // copy_calibration();
    }

    reset_count_x = reset_count;

// Start execution.
#if NRF_LOG_ENABLED
    if (pdPASS != xTaskCreate(logger_thread, "NRF_LOG", 256, NULL, 1, &m_logger_thread)) {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
#endif

    // Activate deep sleep mode.
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    modules_init();
    NRFX_LOG_INFO("Starting: %s %s (%s) FreeRTOS %s\n", DEVICE_NAME, FIRMWARE_REV, MODEL_NUM, tskKERNEL_VERSION_NUMBER);
    NRF_LOG_FLUSH();

    peripherals_init();

    state_machine_init();

    NRF_LOG_FLUSH();

    if (pdPASS != xTaskCreate(monitor_thread, "Monitor", 512, NULL, 1, &m_monitor_thread)) {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

    if (pdPASS != xTaskCreate(ble_thread, "BLE", 256, NULL, 1, &m_ble_thread)) {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

    UNUSED_VARIABLE(xTaskCreate(driver_behaviour_task, "Route", configMINIMAL_STACK_SIZE + 200, NULL, 2, &driver_behaviour_task_handle));

    UNUSED_VARIABLE(xTaskCreate(transfer_task, "Transfer", configMINIMAL_STACK_SIZE + 100, NULL, 2, &transfer_task_handle));

    UNUSED_VARIABLE(xTaskCreate(sampler_task, "Sampler", configMINIMAL_STACK_SIZE + 100, NULL, 3, &sampler_task_handle));

    init_tasks();

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

    terminal_buff_semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(terminal_buff_semaphore);

    sleep_semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(sleep_semaphore);

    command_semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(command_semaphore);

    ble_command_notify_semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(ble_command_notify_semaphore);

    event_semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(event_semaphore);

    flash_semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(flash_semaphore);

    clock_semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(clock_semaphore);

    watchdog_monitor_semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(watchdog_monitor_semaphore);

    flash_counter_semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(flash_counter_semaphore);

    acc_recording_semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(acc_recording_semaphore);

    event_acc_process_sample = xEventGroupCreate();
    event_save_recording     = xEventGroupCreate();
    event_sample_timer       = xEventGroupCreate();
    event_uart_rx            = xEventGroupCreate();
    ble_log_event            = xEventGroupCreate();
    transfer_event           = xEventGroupCreate();
    transfer_confirm_event   = xEventGroupCreate();

    init_ble_task();
}

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    __disable_irq();
    NRF_LOG_FINAL_FLUSH();

#ifndef DEBUG
    NRF_LOG_ERROR("Fatal error");
#else
    switch (id) {
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
    case NRF_FAULT_ID_SD_ASSERT:
        NRF_LOG_ERROR("SOFTDEVICE: ASSERTION FAILED");
        break;
    case NRF_FAULT_ID_APP_MEMACC:
        NRF_LOG_ERROR("SOFTDEVICE: INVALID MEMORY ACCESS");
        break;
#endif
    case NRF_FAULT_ID_SDK_ASSERT: {
        assert_info_t *p_info = (assert_info_t *)info;
        NRF_LOG_ERROR("ASSERTION FAILED at %s:%u", p_info->p_file_name, p_info->line_num);
        break;
    }
    case NRF_FAULT_ID_SDK_ERROR: {
        error_info_t *p_info = (error_info_t *)info;
        NRF_LOG_ERROR("ERROR %u [%s] at %s:%u\r\nPC at: 0x%08x", p_info->err_code, nrf_strerror_get(p_info->err_code), p_info->p_file_name,
                      p_info->line_num, pc);
        NRF_LOG_ERROR("End of error report");
        break;
    }
    default:
        NRF_LOG_ERROR("UNKNOWN FAULT at 0x%08X", pc);
        break;
    }
#endif

    ActivateSoftwareReset(RESET_HARD_FAULT, id, pc, info);

    // NRF_BREAKPOINT_COND;
    // On assert, the system can only recover with a reset.

#ifndef DEBUG
    NRF_LOG_WARNING("System reset");
    NVIC_SystemReset();
#else
    app_error_save_and_stop(id, pc, info);
#endif // DEBUG
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    ActivateSoftwareReset(RESET_STACK_OVERFLOW, 0, 0, 0);
}