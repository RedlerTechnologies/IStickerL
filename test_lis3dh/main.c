#include "FreeRTOS.h"
#include "app_error.h"
#include "app_timer.h"
#include "app_util_platform.h"
#include "drivers/lis3dh.h"
#include "event_groups.h"
#include "hal/hal.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_clock.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_freertos.h"
#include "nrf_strerror.h"
#include "nrfx_log.h"
#include "semphr.h"
#include "task.h"

#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
#include "nrf_sdm.h"
#endif

#include <stdbool.h>
#include <stdint.h>

#if NRF_LOG_ENABLED
static TaskHandle_t m_logger_thread; // Logger thread
#endif

TaskHandle_t driver_behaviour_task_handle;
TaskHandle_t transfer_task_handle;
TaskHandle_t sampler_task_handle;

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

#if NRF_LOG_ENABLED
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
#endif // NRF_LOG_ENABLED

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

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code           = nrf_sdh_ble_default_cfg_set(1, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    // NRF_SDH_BLE_OBSERVER(m_ble_observer, 3, ble_evt_handler, NULL);
}

static TaskHandle_t m_lis3dh_thread;

static void hal_evt_handler(const hal_event_type_t event)
{
    switch (event) {

    case HAL_EVENT_NOTHING:
        NRFX_LOG_INFO("%s HAL_EVENT_NOTHING", __func__);
        break;

    case HAL_EVENT_LIS3DH_INT1:
        xTaskResumeFromISR(m_lis3dh_thread);
        // NRFX_LOG_INFO("%s HAL_EVENT_LIS3DH_INT1", __func__);
        break;

    case HAL_EVENT_LIS3DH_INT2:
        NRFX_LOG_INFO("%s HAL_EVENT_LIS3DH_INT2", __func__);
        break;

    case HAL_EVENT_UART0_RX:
        NRFX_LOG_INFO("%s HAL_EVENT_UART0_RX", __func__);
        break;

    case HAL_EVENT_UART0_TX:
        NRFX_LOG_INFO("%s HAL_EVENT_UART0_TX", __func__);
        break;

    default:
        NRFX_LOG_WARNING("%s Unknown HAL_EVENT %d", __func__, event);
    }
}

static void flash_spi_event_handler(nrfx_spi_evt_t const *p_event, void *p_context)
{
    NRFX_LOG_INFO("%s", __func__);
}

static void peripherals_init(void)
{
    hal_init(hal_evt_handler, flash_spi_event_handler, lis3dh_evt_handler);
    lis3dh_init();

    NRF_LOG_FLUSH();

    hal_interrupts_set(true, true);
}

void lis3dh_thread(void *pvParameters)
{
    UNUSED_PARAMETER(pvParameters);

    lis3dh_configure_fifo();

    while (1) {
        vTaskSuspend(NULL);

        lis3dh_int_handler();
    }
}

/**@brief Function for starting advertising. */
static void advertising_start(void *context)
{
    // ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    // APP_ERROR_CHECK(err_code);
}

int main(void)
{
    // Initialize.
    log_init();
    clock_init();
    ble_stack_init();

// Start execution.
#if NRF_LOG_ENABLED
    if (pdPASS != xTaskCreate(logger_thread, "NRF_LOG", 256, NULL, 1, &m_logger_thread)) {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
#endif

    // Activate deep sleep mode.
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    modules_init();
    NRFX_LOG_INFO("Starting LIS3DH Tester FreeRTOS %s\n", tskKERNEL_VERSION_NUMBER);
    NRF_LOG_FLUSH();

    peripherals_init();

    NRF_LOG_FLUSH();

    if (pdPASS != xTaskCreate(lis3dh_thread, "LIS3DH", 512, NULL, 1, &m_lis3dh_thread)) {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

    NRFX_LOG_INFO("%s Free Heap: %u", __func__, xPortGetFreeHeapSize());

    nrf_sdh_freertos_init(advertising_start, NULL);

    // Start FreeRTOS scheduler.
    vTaskStartScheduler();

    NRF_LOG_FLUSH();

    for (;;) {
        APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
    }
}
