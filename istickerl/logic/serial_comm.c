#include "serial_comm.h"

#include "FreeRTOS.h"
#include "hal/hal_drivers.h"
#include "task.h"

#define NRF_LOG_MODULE_NAME logic_serial_comm
#define NRF_LOG_LEVEL CLOUD_WISE_DEFAULT_LOG_LEVEL
#include "nrfx_log.h"
NRF_LOG_MODULE_REGISTER();

static TaskHandle_t m_uart_thread;

static void uart_thread(void *arg);

void serial_comm_init(void)
{
    if (pdPASS != xTaskCreate(uart_thread, "UART", 256, NULL, 1, &m_uart_thread)) {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
}

static void uart_thread(void *arg)
{
    UNUSED_PARAMETER(arg);

    ret_code_t err_code;

    static uint8_t data[1];

    while (1) {
        err_code = nrfx_uart_rx(hal_uart, data, sizeof(data));
        APP_ERROR_CHECK(err_code);

        vTaskSuspend(NULL);

        NRFX_LOG_INFO("%s RX %c", __func__, data[0]);
    }
}

void serial_comm_send_text(char tx_data[])
{
    ret_code_t  err_code;

    err_code = nrfx_uart_tx(hal_uart, tx_data, strlen(tx_data));

    APP_ERROR_CHECK(err_code);
}

void serial_comm_process_rx(void)
{
    //
    xTaskResumeFromISR(m_uart_thread);
}