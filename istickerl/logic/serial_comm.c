#include "serial_comm.h"

#include "FreeRTOS.h"
#include "hal/hal_drivers.h"
#include "logic/commands.h"
#include "semphr.h"
#include "task.h"

#define NRF_LOG_MODULE_NAME logic_serial_comm
#define NRF_LOG_LEVEL CLOUD_WISE_DEFAULT_LOG_LEVEL

#define UART_RX_BUFFER_SIZE 64

xSemaphoreHandle tx_uart_semaphore;

#include "nrfx_log.h"
NRF_LOG_MODULE_REGISTER();

static TaskHandle_t m_uart_thread;

static void uart_thread(void *arg);

static uint8_t rx_buffer[UART_RX_BUFFER_SIZE];
static uint8_t copy_rx_buffer[UART_RX_BUFFER_SIZE + 4];

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
    static uint8_t rx_ptr      = 0;
    static char    crlf_data[] = "\r\n";

    while (1) {
        err_code = nrfx_uart_rx(hal_uart, data, sizeof(data));
        APP_ERROR_CHECK(err_code);

        if (data[0] == 0x00) {
            vTaskSuspend(NULL);
            continue;
        }

        if (rx_ptr == 0)
            memset(rx_buffer, 0x00, UART_RX_BUFFER_SIZE);

        rx_buffer[rx_ptr] = data[0];

        // echo
        err_code = nrfx_uart_tx(hal_uart, data, 1);
        // vTaskDelay(10);

        if (rx_ptr < UART_RX_BUFFER_SIZE)
            rx_ptr++;

        if (data[0] == '\r') {
            memcpy(copy_rx_buffer, rx_buffer, rx_ptr);
            // err_code = nrfx_uart_tx(hal_uart, copy_rx_buffer, strlen(copy_rx_buffer));

            copy_rx_buffer[rx_ptr]     = '\r';
            copy_rx_buffer[rx_ptr + 1] = '\n';

            DisplayMessage("\r\n\r\n\r\n", 6);
            DisplayMessage(copy_rx_buffer, 0);
            command_decoder(copy_rx_buffer, rx_ptr, 0);
            vTaskDelay(10);
            err_code = nrfx_uart_tx(hal_uart, crlf_data, strlen(crlf_data));
            vTaskDelay(10);
            rx_ptr = 0;

            // run command handler
            // ..
        }

        vTaskSuspend(NULL);

        NRFX_LOG_INFO("%s RX %c", __func__, data[0]);
    }
}

void serial_comm_send_text(void)
{
    static char tx_data[] = "Hello\r\n";
    ret_code_t  err_code;

    // err_code = nrfx_uart_tx(hal_uart, tx_data, strlen(tx_data));

    // APP_ERROR_CHECK(err_code);
}

void serial_comm_process_rx(void)
{
    //
    xTaskResumeFromISR(m_uart_thread);
}

void DisplayMessage(uint8_t *message, uint8_t len)
{
    xSemaphoreTake(tx_uart_semaphore, portMAX_DELAY);
    vTaskDelay(10);

    if (len == 0)
        len = strlen(message);

    nrfx_uart_tx(hal_uart, message, len);

    xSemaphoreGive(tx_uart_semaphore);
}