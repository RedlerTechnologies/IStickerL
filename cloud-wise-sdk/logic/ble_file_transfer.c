#include "ble_file_transfer.h"

#include "FreeRTOS.h"
#include "ble/ble_services_manager.h"
#include "drivers/flash.h"
#include "events.h"
#include "flash_data.h"
#include "hal/hal_boards.h"
#include "logic/serial_comm.h"
#include "nrf_delay.h"
#include "recording.h"
#include "semphr.h"

#include <string.h>

#define NRF_LOG_MODULE_NAME ble_file_transfer
#define NRF_LOG_LEVEL CLOUD_WISE_DEFAULT_LOG_LEVEL
#include "nrfx_log.h"
NRF_LOG_MODULE_REGISTER();

#define BLE_READ_BUFFER_FILE_SIZE 256

#define BLE_STATUS_SUCCESS 0

static uint8_t ble_file_read_packet[BFT_PACKET_SIZE];
static uint8_t ble_file_read_buffer_t[BLE_READ_BUFFER_FILE_SIZE + 4];
uint8_t *      ble_file_read_buffer;

BleReadingFileState ble_reading_file_state;

extern DriverBehaviourState driver_behaviour_state;
extern AccRecord acc_record;

static uint32_t GetAbsoluteReadAddress(unsigned packet_index)
{
    //	packet_index is in multiple of 16

    uint32_t sum_packets      = 0;
    uint32_t absolute_address = 0;
    uint8_t  i;

    for (i = 0; i < ble_reading_file_state.num_of_regions; i++) {
        sum_packets += (ble_reading_file_state.reg_size[i] * BLE_READ_BUFFER_FILE_SIZE);

        if ((packet_index * 16) < sum_packets) {
            absolute_address = ble_reading_file_state.reg_start_address[i] * BLE_READ_BUFFER_FILE_SIZE;
            absolute_address += packet_index * 16;
            break;
        }

        packet_index -= ble_reading_file_state.reg_size[i] * BLE_READ_BUFFER_FILE_SIZE / 16;
    }

    return absolute_address;
}

void FileTransferFailed(bool abort)
{
    if (ble_services_is_connected() & !abort) {
        if (ble_reading_file_state.record_num >= 0)
            record_write_status(ble_reading_file_state.record_num, RECORD_SENT_IND, 1 /* error status>0 */);
    }

    ble_reading_file_state.next_read_address = 0x0FFFFFFF;
    ble_reading_file_state.state             = 0xFF;

    if (ble_reading_file_state.record_num >= 0)
        DisplayMessage("\r\nFile transfer aborted\r\n", 0, true);
    // CreateLogEvent(LOG_BLE_READ_FILE_ABORTED, 2);

    ble_reading_file_state.record_num = -1;
}

bool in_sending_file(void)
{
    // check is during sending file process
    return (ble_reading_file_state.state != 0xFF);
}

static void FileTransferSucceed(void)
{
    if (ble_reading_file_state.record_num >= 0)
        record_write_status(ble_reading_file_state.record_num, RECORD_SENT_IND, 0);

    ble_reading_file_state.state      = 0xFF;
    ble_reading_file_state.record_num = -1;

    acc_record.last_sent_record_num_count++;

    DisplayMessage("\r\nFile transfer succeed\r\n", 0, true);
    // CreateLogEvent(LOG_BLE_READ_FILE_COMPLETED, 2);

    /*
        if (ble_reading_file_state.file_type == FILE_TYPE_PARAMS)
            error_bits.configuration_Changed = 0;
            */
}

void BFT_send_next_packet(void)
{
    // BleEvent * ble_event;
    uint32_t   packet_index = 0;
    uint32_t   duration;
    uint32_t   abs_address;
    uint8_t    retry_count;
    ret_code_t err_code;
    bool       ret_ble_error;

    memset(ble_file_read_packet, 0x00, BFT_PACKET_SIZE);

    // ble_event = &ble_event_list[BLE_FILEREAD];

    if (ble_reading_file_state.next_read_address >= ble_reading_file_state.file_length /* GetFileMaxAddress() */) {
        // send file termination

        ble_file_read_packet[0] = 0xFF;
        ble_file_read_packet[1] = 0xFF;
        ble_file_read_packet[2] = 0xFF;

        memcpy(ble_file_read_packet + 4, (unsigned char *)(&ble_reading_file_state.crc), 4);

        // ret = aci_gatt_update_char_value(stickerServHandle, ble_event->handle, 0, 20, ble_file_read_packet);
        // if (ret != BLE_STATUS_SUCCESS)
        //    return;

        ret_ble_error = !ble_services_notify_file_transfer(ble_file_read_packet, BFT_PACKET_SIZE);
        // ret_ble_error = !ble_services_notify_event(ble_file_read_packet, BFT_PACKET_SIZE);

#ifdef SIMULATE_TRANSFER
        ret_ble_error = false;
#else
        if (ret_ble_error)
            return;
#endif

        vTaskDelay(100);

        if (driver_behaviour_state.new_transfer_protocol) {
            ble_reading_file_state.next_read_address -= BLE_READ_BUFFER_FILE_SIZE;
            ble_reading_file_state.state = 1;

#ifdef SIMULATE_TRANSFER
#ifdef SIMULATE_WITH_POSITIVE_ACK
            vTaskDelay(1000);
            ble_reading_file_state.state = 2;
#endif
#endif

            ble_reading_file_state.last_ack_time = xTaskGetTickCount();
            return;
        } else {
            if (ble_reading_file_state.retries > 0) {
                FileTransferFailed(false);
            } else {
                FileTransferSucceed();
            }

            ble_reading_file_state.state      = 0xFF;
            ble_reading_file_state.record_num = -1;
            return;
        }
    } else if (ble_reading_file_state.next_read_address == 0) {
        // send file start message

        DisplayMessage("BLE: File read Started", 0, true);
        // CreateLogEvent(LOG_BLE_READ_FILE_STARTED, 2);

        //        memcpy(ble_file_read_packet + 4, ble_reading_file_state.file_length, 4); <-- a bug
        //        ret = aci_gatt_update_char_value(stickerServHandle, ble_event->handle, 0, 20, ble_file_read_packet);

        memcpy(ble_file_read_packet + 4, (unsigned char *)&ble_reading_file_state.file_length, 4);
        ret_ble_error = !ble_services_notify_file_transfer(ble_file_read_packet, BFT_PACKET_SIZE);
        // ret_ble_error = !ble_services_notify_event(ble_file_read_packet, BFT_PACKET_SIZE);

#ifdef SIMULATE_TRANSFER
        ret_ble_error = false;

        DisplayMessage("\r\nstart block\r\n", 0);
        vTaskDelay(FILE_DELAY);
#endif

        vTaskDelay(100);
    }

    packet_index = (ble_reading_file_state.next_read_address / 16);

    if ((packet_index % 16) == 0) {
        if (packet_index > 0) {
            switch (ble_reading_file_state.state) {
            case 0:
                // send crc
                ble_file_read_packet[2]      = 0x80;
                ble_reading_file_state.state = 1;

#ifdef SIMULATE_TRANSFER
#ifdef SIMULATE_WITH_POSITIVE_ACK
                vTaskDelay(1000);
                ble_reading_file_state.state = 2;
#endif
#endif

                ble_reading_file_state.last_ack_time = xTaskGetTickCount();

                if (driver_behaviour_state.new_transfer_protocol)
                    ble_file_read_packet[3] = 2;

                memcpy(ble_file_read_packet + 4, (unsigned char *)(&ble_reading_file_state.last_packet_crc), 4);

                // ret = aci_gatt_update_char_value(stickerServHandle, ble_event->handle, 0, 20, ble_file_read_packet);
                ret_ble_error = !ble_services_notify_file_transfer(ble_file_read_packet, BFT_PACKET_SIZE);
                // ret_ble_error = !ble_services_notify_event(ble_file_read_packet, BFT_PACKET_SIZE);

#ifdef SIMULATE_TRANSFER
                ret_ble_error = false;

                terminal_buffer_lock();
                sprintf(alert_str, "\r\nCRC: %d\r\n", ble_reading_file_state.last_packet_crc);
                DisplayMessage(alert_str, 0);
                terminal_buffer_release();
                vTaskDelay(FILE_DELAY);
#endif
                return;

                break;

            case 1:
            case 3:
                // waiting for ack

                if (ble_reading_file_state.state == 1) {
                    duration = timeDiff(xTaskGetTickCount(), ble_reading_file_state.last_ack_time);

#ifdef SIMULATE_TRANSFER
                    if (duration < 30000) {
#else
                    if (duration < 3000) {
#endif
                        if (ble_reading_file_state.next_read_address < 0)
                            ble_reading_file_state.next_read_address = 0;

                        return;
                    }
                }

                ble_reading_file_state.retries++;
                ble_reading_file_state.next_read_address = ble_reading_file_state.prev_packet_address;
                ble_reading_file_state.crc               = ble_reading_file_state.prev_crc;
                ble_reading_file_state.state             = 0;

                if (ble_reading_file_state.retries >= 5) {
                    FileTransferFailed(false);
                }

                if (driver_behaviour_state.new_transfer_protocol) {
                    packet_index = (ble_reading_file_state.next_read_address / 16);
                    break;
                }

                return;

                break;

            case 2:
                ble_reading_file_state.prev_packet_address += BLE_READ_BUFFER_FILE_SIZE;
                ble_reading_file_state.retries  = 0;
                ble_reading_file_state.state    = 0;
                ble_reading_file_state.prev_crc = ble_reading_file_state.crc;

                if (driver_behaviour_state.new_transfer_protocol) {
                    if (ble_reading_file_state.prev_packet_address >= ble_reading_file_state.file_length) {
                        FileTransferSucceed();
                        return;
                    }
                }

                break;
            }
        }

        // read file data from FLASH

        ble_reading_file_state.offset = 0;
        abs_address                   = GetAbsoluteReadAddress(packet_index);

        flash_read_buffer(ble_file_read_buffer_t, abs_address, BLE_READ_BUFFER_FILE_SIZE);
        ble_file_read_buffer = ble_file_read_buffer_t + 4;

        ble_reading_file_state.crc             = CRC16_Calc(ble_file_read_buffer, BLE_READ_BUFFER_FILE_SIZE, ble_reading_file_state.crc);
        ble_reading_file_state.last_packet_crc = CRC16_Calc(ble_file_read_buffer, BLE_READ_BUFFER_FILE_SIZE, 0);

        terminal_buffer_lock();
        sprintf(alert_str, "\r\nBLE: read file address: %d/%d\r\n",
                (ble_reading_file_state.prev_packet_address + BLE_READ_BUFFER_FILE_SIZE), ble_reading_file_state.file_length);
        DisplayMessage(alert_str, 0, false);
        terminal_buffer_release();
    }

    packet_index++;

    memset(ble_file_read_packet, 0x00, 20);
    ble_file_read_packet[0] = (packet_index & 0xFF);
    ble_file_read_packet[1] = (packet_index & 0xFF00) >> 8;
    ble_file_read_packet[2] = (packet_index & 0x7F0000) >> 16;

    memcpy(ble_file_read_packet + 4, ble_file_read_buffer + ble_reading_file_state.offset, 16);

    ret_ble_error = true;
    retry_count   = 0;

    while (ret_ble_error && retry_count < 3) {
        if (driver_behaviour_state.new_transfer_protocol)
            ble_file_read_packet[3] = 16;

        // send data packet
        // ret = aci_gatt_update_char_value(stickerServHandle, ble_event->handle, 0, 20, ble_file_read_packet);
        ret_ble_error = !ble_services_notify_file_transfer(ble_file_read_packet, BFT_PACKET_SIZE);
        // ret_ble_error = !ble_services_notify_event(ble_file_read_packet, BFT_PACKET_SIZE);

#ifdef SIMULATE_TRANSFER
        terminal_buffer_lock();
        sprintf(alert_str, "\r\ndata: %d\r\n", packet_index);
        DisplayMessageWithNoLock(alert_str, 0);
        ret_ble_error = false;
        terminal_buffer_release();
        vTaskDelay(FILE_DELAY);
#else
        vTaskDelay(50);
#endif

        if (!ret_ble_error) {
            ble_reading_file_state.next_read_address += 16;
            ble_reading_file_state.offset += 16;
        } else {
            // DisplayChar('T');
        }
        retry_count++;
    }

    if (ret_ble_error)
        packet_index = 0;
}

void BFT_start(unsigned short record_num, unsigned char is_record)
{
    unsigned      flash_address;
    unsigned char i = 0;

    if (is_record) {
        ble_reading_file_state.num_of_regions = 1;
        ble_reading_file_state.record_num     = record_num;

        flash_address                               = FLASH_RECORDS_START_ADDRESS + (record_num)*RECORD_SIZE;
        ble_reading_file_state.reg_start_address[0] = flash_address / BLE_READ_BUFFER_FILE_SIZE;
        ble_reading_file_state.reg_size[0]          = RECORD_SIZE / BLE_READ_BUFFER_FILE_SIZE;
    } else {
        ble_reading_file_state.num_of_regions       = 2;
        ble_reading_file_state.reg_start_address[0] = FLASH_DEVICE_PARAMS_ADDRESS / BLE_READ_BUFFER_FILE_SIZE;
        ble_reading_file_state.reg_size[0]          = 2;
        ble_reading_file_state.reg_start_address[1] = FLASH_APP_PARAMS_ADDRESS / BLE_READ_BUFFER_FILE_SIZE;
        ble_reading_file_state.reg_size[1]          = 2;
    }

    ble_reading_file_state.next_read_address   = 0;
    ble_reading_file_state.prev_packet_address = 0;
    ble_reading_file_state.crc                 = 0;
    ble_reading_file_state.prev_crc            = 0;
    ble_reading_file_state.last_packet_crc     = 0;
    // ble_reading_file_state.state               = 0;
    ble_reading_file_state.retries = 0;

    ble_reading_file_state.file_length = 0;

    for (i = 0; i < ble_reading_file_state.num_of_regions; i++)
        ble_reading_file_state.file_length += ble_reading_file_state.reg_size[i];

    ble_reading_file_state.file_length *= BLE_READ_BUFFER_FILE_SIZE;

    ble_reading_file_state.state = 0;
}