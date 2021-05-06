#include "serial_comm.h"

#include "FreeRTOS.h"
#include "event_groups.h"
#include "hal/hal_drivers.h"
#include "logic/commands.h"
#include "logic/monitor.h"
#include "nrf_delay.h"
#include "semphr.h"
#include "task.h"

#define NRF_LOG_MODULE_NAME logic_serial_comm
#define NRF_LOG_LEVEL CLOUD_WISE_DEFAULT_LOG_LEVEL

#define UART_RX_BUFFER_SIZE 64

xSemaphoreHandle   tx_uart_semaphore;
EventGroupHandle_t event_uart_rx;

uint8_t alert_str[256 + 16];

#include "nrfx_log.h"
NRF_LOG_MODULE_REGISTER();

static TaskHandle_t m_uart_thread;

static void uart_thread(void *arg);

static uint8_t copy_rx_buffer[UART_RX_BUFFER_SIZE + 4];

static uint8_t rx_buffer[UART_RX_BUFFER_SIZE];
static uint8_t rx_ptr;

xSemaphoreHandle terminal_buff_semaphore;

void serial_comm_init(void)
{
    if (pdPASS != xTaskCreate(uart_thread, "UART", 256, NULL, 1, &m_uart_thread)) {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
}

static void uart_thread(void *arg)
{
    UNUSED_PARAMETER(arg);

    static uint8_t result_buffer[64];

    ret_code_t  err_code;
    EventBits_t uxBits;

    static uint8_t data[1];
    static uint8_t rx_ptr      = 0;
    static char    crlf_data[] = "\r\n";

    err_code = nrfx_uart_rx(hal_uart, data, sizeof(data));

    while (1) {

        monitor_task_set(TASK_MONITOR_BIT_UART_RX);

        uxBits = xEventGroupWaitBits(event_uart_rx, 0x01, pdTRUE, pdFALSE, 5);

        if (uxBits) {
            terminal_buffer_lock();
            DisplayMessage(copy_rx_buffer, 0, false);
            terminal_buffer_release();
            command_decoder(copy_rx_buffer, strlen(copy_rx_buffer), result_buffer, 0);
        }
    }

    vTaskSuspend(NULL);

    // NRFX_LOG_INFO("%s RX %c", __func__, data[0]);
}

void serial_comm_process_rx(void)
{
    static uint8_t echo = 0;

    BaseType_t xHigherPriorityTaskWoken, xResult;

    uint8_t ch;

    ret_code_t err_code;
    uint8_t    data[1];

    err_code = nrfx_uart_rx(hal_uart, data, sizeof(data));
    APP_ERROR_CHECK(err_code);

    ch = data[0];

    if (rx_ptr == 0) {
        memset(rx_buffer, 0x00, UART_RX_BUFFER_SIZE);

        if (ch == 'X') {
            echo = 1;
            return;
        }
    }

    rx_buffer[rx_ptr] = ch;

    if (echo)
        nrfx_uart_tx(hal_uart, &ch, 1);

    if (ch == 0x0d) {
        memcpy(copy_rx_buffer, rx_buffer, UART_RX_BUFFER_SIZE);
        memset(rx_buffer, 0x00, UART_RX_BUFFER_SIZE);
        rx_ptr = 0;
        echo = 0;

        xHigherPriorityTaskWoken = pdFALSE;
        xResult                  = xEventGroupSetBitsFromISR(event_uart_rx, 0x01, &xHigherPriorityTaskWoken);

        if (xResult != pdFAIL) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }

    } else {
        if (rx_ptr + 1 < UART_RX_BUFFER_SIZE)
            rx_ptr++;
    }
}

void DisplayMessage(uint8_t *message, uint8_t len, bool with_lock)
{
    if (with_lock) {
        if (!xSemaphoreTake(tx_uart_semaphore, 50))
            return;
    }

    while (nrfx_uart_tx_in_progress(hal_uart)) {
    }

    if (len == 0)
        len = strlen(message);

    nrfx_uart_tx(hal_uart, message, len);

    if (with_lock)
        xSemaphoreGive(tx_uart_semaphore);
}

void DisplayMessageWithTime(uint8_t *message, uint8_t len, bool with_lock)
{
    static uint8_t time_buffer[20];
    uint8_t        len1;

    if (with_lock) {
        if (!xSemaphoreTake(terminal_buff_semaphore /*tx_uart_semaphore*/, 50))
            return;
    }

    while (nrfx_uart_tx_in_progress(hal_uart)) {
    }

    SetClockString(time_buffer);
    len1 = strlen(time_buffer);
    nrfx_uart_tx(hal_uart, time_buffer, len1);

    while (nrfx_uart_tx_in_progress(hal_uart)) {
    }

    while (nrfx_uart_tx_in_progress(hal_uart)) {
    }

    if (len == 0)
        len = strlen(message);

    nrfx_uart_tx(hal_uart, message, len);

    if (with_lock)
        xSemaphoreGive(tx_uart_semaphore);
}

void terminal_buffer_lock(void) { xSemaphoreTake(terminal_buff_semaphore, portMAX_DELAY); }

void terminal_buffer_release(void) { xSemaphoreGive(terminal_buff_semaphore); }