#include "ble_task.h"

#include "FreeRTOS.h"
#include "ble/ble_services_manager.h"
#include "ble_istickerl.h"
#include "logic/ble_file_transfer.h"
#include "logic/commands.h"
#include "logic/serial_comm.h"
#include "nrf_log_ctrl.h"
#include "nrf_ringbuf.h"
#include "nrfx_atomic.h"
#include "queue.h"

#define NRF_LOG_MODULE_NAME ble_task
#define NRF_LOG_LEVEL BLE_ISTICKERL_BLE_LOG_LEVEL
#include "nrfx_log.h"
NRF_LOG_MODULE_REGISTER();


extern BleReadingFileState ble_reading_file_state;

static QueueHandle_t ble_command_queue;

void init_ble_task(void) { ble_command_queue = xQueueCreate(1, sizeof(BleMessage *)); }

bool PostBleCommand(uint8_t *command_str, uint8_t size)
{
    static BleMessage ble_message;

    // NRFX_LOG_INFO("command: size:%d, <%s>", size, command_str);

    BleMessage *p;

    if (xQueuePeek(ble_command_queue, &p, 0) == pdFALSE) {
        memset(ble_message.message, 0x00, COMMAND_CHAR_MAX_LEN);
        memcpy(ble_message.message, command_str, size - 1);

        NRFX_LOG_INFO("command: size:%d, <%s>", size, ble_message.message);

        ble_message.size = size;
        p                = &ble_message;

        NRFX_LOG_INFO("command pushed");
        xQueueSendFromISR(ble_command_queue, &p, NULL);
    } else {
        NRFX_LOG_INFO("command queue is full");
    }
}

void ble_thread(void *pvParameters)
{
    static uint8_t result_buffer[COMMAND_CHAR_MAX_LEN];
    BleMessage *   p;

    ble_reading_file_state.state = 0xFF;

    while (1) {

        if (xQueuePeek(ble_command_queue, &p, 0) == pdTRUE) {

            DisplayMessage(p->message, p->size);
            DisplayMessage("\r\n", 2);

            NRFX_LOG_INFO("command pop-up size=%d <%s>", p->size, p->message);

            memset(result_buffer, 0x00, COMMAND_CHAR_MAX_LEN);
            command_decoder(p->message, p->size, result_buffer + 2, 1);

            NRFX_LOG_INFO("after decoder");
            xQueueReceive(ble_command_queue, &p, 0);
            NRFX_LOG_INFO("after release");

            result_buffer[0] = 0x80;
            result_buffer[1] = strlen(result_buffer + 2);
            ble_services_notify_command(result_buffer, strlen(result_buffer));

            NRFX_LOG_INFO("result: <%s>", result_buffer + 2);
        }

        if (ble_reading_file_state.state < 0xFF) {
            BFT_send_next_packet();
        }
    }
}

void PostBleAlert(uint8_t *command_str)
{
    uint8_t len = strlen(command_str + 2);

    // send to terminal with CRLF
    // sent to BLE without CRLF !!!

    DisplayMessage(command_str + 2, 0);

    command_str[0] = 0x80;
    command_str[1] = len - 2;

#ifdef BLE_ADVERTISING
    // this command need semaphore
    ble_services_notify_command(command_str, len);
#endif
}