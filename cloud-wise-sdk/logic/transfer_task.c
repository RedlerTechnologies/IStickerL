#include "transfer_task.h"

#include "FreeRTOS.h"
#include "app_timer.h"
#include "ble/ble_istickerl.h"
#include "ble/ble_services_manager.h"
#include "commands.h"
#include "configuration.h"
#include "decoder.h"
#include "drivers/buzzer.h"
#include "event_groups.h"
#include "events.h"
#include "flash_data.h"
#include "hal/hal_boards.h"
#include "hal/hal_drivers.h"
#include "logic/peripherals.h"
#include "logic/serial_comm.h"
#include "logic/state_machine.h"
#include "monitor.h"
#include "recording.h"
#include "semphr.h"
#include "task.h"
#include "tracking_algorithm.h"

#include <stdlib.h>
#include <string.h>

EventGroupHandle_t transfer_event;
EventGroupHandle_t transfer_confirm_event;

extern xSemaphoreHandle     event_semaphore;
extern ScanResult           scan_result;
extern DeviceConfiguration  device_config;
extern DriverBehaviourState driver_behaviour_state;

#define COMMAND_CONNECT_SEND_EVENTS 0x0
#define COMMAND_SEND_CRC 0x1
#define COMMAND_CONNECT_CONFIGURATION 0x2
#define COMMAND_CLOSE 0x3

uint8_t  SendingEventData(unsigned char ble_mode);
uint16_t Build_R2_Packet(uint8_t cmd, uint8_t flags, uint8_t *sending_buffer, uint32_t value, uint16_t crc, uint8_t device_len);
bool     send_data(uint8_t *data, uint8_t len);

static uint8_t transfer_buffer[256 + 16];

static void AddBLEHeader(uint8_t *buffer, uint32_t packet_count) { memcpy(buffer, (char *)&packet_count, 4); }

void transfer_task(void *pvParameter)
{
    EventBits_t result;
    uint8_t error;

    while (1) {

        xEventGroupClearBits(transfer_event, 0xFF);
        result = xEventGroupWaitBits(transfer_event, EVENT_BLE_CONNECTION, pdTRUE, pdFALSE, (TickType_t)1000);

        monitor_task_set(TASK_MONITOR_BIT_TRANSFER);

        if (result & EVENT_BLE_CONNECTION) {

            driver_behaviour_state.in_event_transfer_process = true;

            error = SendingEventData(1);

            driver_behaviour_state.in_event_transfer_process = false;

            if (error) {
                terminal_buffer_lock();
                sprintf(alert_str, (const char *)"Event transmission session failed with error: %d\r\n", error);
                DisplayMessage(alert_str, 0, false);
                terminal_buffer_release();

                continue;
            }
        }
    }
}

static uint16_t tx_buffer_size;
static uint32_t packet_count;

static FlashMarker  temp_marker;
static xFlashMarker last_valid_marker;
static xFlashMarker before_write_marker;

uint8_t SendingEventData(unsigned char ble_mode)
{
    uint32_t    packet_to_send_count = 0;
    uint16_t    event_count          = 0;
    uint16_t    tx_msg_len;
    uint16_t    sector_id1;
    uint16_t    sector_id2;
    uint16_t    crc = 0xFFFF;
    uint16_t    timeout;
    EventBits_t ble_transfer_confirmation;
    uint8_t     error = 0;
    uint8_t     e2;
    uint8_t     sucessive_nodata_count = 0;
    uint8_t     data_found             = 0;
    uint8_t     commited               = 0;
    uint8_t     completed              = 0;
    uint8_t     end_session_sent       = 0;
    uint8_t     flags;
    bool        ret_ble_error;

    // tx_buffer_size = 256;
    // tx_buffer_size = 196;
    tx_buffer_size = 112;
    packet_count   = 0;

    ///////////////////////////////////////
    // sending the R2 connection message //
    ///////////////////////////////////////

    memset(transfer_buffer, 0, tx_buffer_size);

    xEventGroupClearBits(transfer_confirm_event, 0xFF);
    xEventGroupClearBits(transfer_event, 0xFF);

    tx_msg_len = Build_R2_Packet(COMMAND_CONNECT_SEND_EVENTS, 0, transfer_buffer + 6, 0, 0, 15);
    transfer_buffer[0] = 0x80;
    transfer_buffer[1] = tx_msg_len + 4;
    memcpy(transfer_buffer + 2, &packet_count, 4);
    send_data(transfer_buffer, tx_msg_len + 6);

    // validate the connection response message
    // ..

    // save markers, in case where event data transfer fails //

    if (xSemaphoreTake(event_semaphore, (portTickType)500)) {

        before_write_marker.event_id      = scan_result.write_marker.event_id;
        before_write_marker.flash_address = scan_result.write_marker.flash_address;

        // display read marker
        terminal_buffer_lock();
        sprintf(alert_str, "read ptr before: 0x%x %d\r\n", scan_result.read_marker.event_id, scan_result.read_marker.flash_address);
        DisplayMessage(alert_str, 0, false);
        terminal_buffer_release();

        scan_result.previous_event_id = 0;
        scan_result.missing_event_id  = 0;

        xSemaphoreGive(event_semaphore);
    } else
        error = DATA_UPLOAD_ERROR_FLASH;

    temp_marker                = scan_result.read_marker;
    last_valid_marker.event_id = 0;

    if (scan_result.read_marker.flash_address == before_write_marker.flash_address)
        completed = 1;

    if (!error) {
        while (!completed) {
            vTaskDelay(10);

#ifdef BLE_ADVERTISING
            if (ble_mode) {
                if (!ble_services_is_connected()) {
                    error = DATA_UPLOAD_ERROR_BLE_DISCONNCTED;
                    DisplayMessage("Transfer Aborted cause BLE disconnected!", 0, true);
                    break;
                }
            }
#endif
            flags = (MARKER_OPT_GET_CONTINUE | MARKER_OPT_IGNORE_CRC_ERROR);
            if (driver_behaviour_state.print_sent_event_id_mode)
                flags |= MARKER_OPT_PRINT_ID;

            if (xSemaphoreTake(event_semaphore, (portTickType)500) == pdTRUE) {
                InitMarker(&temp_marker, transfer_buffer + 6, temp_marker.flash_address, tx_buffer_size - 6, 0, flags);

                temp_marker.limit_address = before_write_marker.flash_address;

                ReadEventsToMarker(&temp_marker);
                xSemaphoreGive(event_semaphore);

                event_count += temp_marker.event_count;

                if (temp_marker.event_count > 0) {
                    data_found = 1;
                    packet_count++;

                    last_valid_marker.flash_address = temp_marker.flash_address - temp_marker.event_size;
                    last_valid_marker.event_id      = temp_marker.event_id;

                    transfer_buffer[0] = 0x80; // (packet_count & 0x7F); // | 0x80;
                    transfer_buffer[1] = temp_marker.offset + 4;
                    memcpy(transfer_buffer + 2, &packet_count, 4);
                    send_data(transfer_buffer, temp_marker.offset + 6);

                    vTaskDelay(50);

                    packet_to_send_count++;

                    // display packet counter to terminal

                    if ((packet_to_send_count % 10) == 0) {
                        terminal_buffer_lock();
                        sprintf(alert_str, "\r\nPacket Sent: %d\r\n", packet_to_send_count);
                        DisplayMessage(alert_str, 0, false);
                        terminal_buffer_release();
                    }

                    if (temp_marker.flash_address == before_write_marker.flash_address) {
                        completed = 1;
                    }

                } else {
                    // no data
                    sucessive_nodata_count++;

                    sector_id1 = GetSectorID(temp_marker.flash_address);
                    sector_id2 = GetSectorID(before_write_marker.flash_address);

                    if (sector_id2 != sector_id1) {
                        sector_id1++;
                        CheckSectorLimit(&sector_id1);
                        temp_marker.flash_address = GetFlashAddress(sector_id1);
                    } else {
                        completed = 1;
                    }
                }

                /////////////////////////////////////////////
                // checking for transfer abort from client //
                // or complete reason                      //
                /////////////////////////////////////////////

                ble_transfer_confirmation = xEventGroupWaitBits(transfer_event, EVENT_BLE_TRANSFER_ABORT, pdFALSE, pdFALSE, (TickType_t)1);

                if (ble_transfer_confirmation & EVENT_BLE_TRANSFER_ABORT) {
                    DisplayMessage("BLE transfer aborted by application", 0, true);
                    error = DATA_UPLOAD_ERROR_ABORTED_BY_APP;
                    break;
                }

                if (ble_transfer_confirmation & EVENT_BLE_BLOCK_CONFIRM_AND_EXIT) {
                    completed = 1;
                }

                if (sucessive_nodata_count >= 64) {
                    completed = 1;
                }

                if (ble_mode) {
                    if (packet_to_send_count >= 150 /* 100*/ )
                        completed = 1;
                }
            } else {
                // can not take semaphore
            }
        }

        if (packet_to_send_count > 0) {
            terminal_buffer_lock();
            sprintf(alert_str, "\r\nPacket Sent: %d\r\n", packet_to_send_count);
            DisplayMessage(alert_str, 0, false);
            terminal_buffer_release();
        }

        // sending disconnection message //

        if (!error) {
            packet_count++;

            // build the disconnection message
            memset(transfer_buffer, 0, tx_buffer_size);

            // send the disconnection message

            flags = 0;

            tx_msg_len = Build_R2_Packet(COMMAND_SEND_CRC, 0, transfer_buffer + 6, (int)event_count, crc, 15);
            transfer_buffer[0] = 0x80;
            transfer_buffer[1] = tx_msg_len + 4;
            memcpy(transfer_buffer + 2, &packet_count, 4);
            send_data(transfer_buffer, tx_msg_len + 6);

            end_session_sent = 1;

            ///////////////////////////
            // wait for confirmation //
            ///////////////////////////

            ble_transfer_confirmation =
                xEventGroupWaitBits(transfer_confirm_event, 0xFF, pdFALSE, pdFALSE, (TickType_t)20000 /*(TickType_t)10000*/);

            if (ble_transfer_confirmation & EVENT_BLE_TRANSFER_SUCCESS)
                error = 0;
            else if (ble_transfer_confirmation & EVENT_BLE_TRANSFER_ERROR)
                error = DATA_UPLOAD_ERROR_DISCONNECTION_MSG;
            else
                error = DATA_UPLOAD_ERROR_CONNECTION_TIMEOUT;
        }

        // update markers (roll-back or commit)

        if (xSemaphoreTake(event_semaphore, (portTickType)500) == pdTRUE) {
            if (!data_found) {
                DisplayMessage("NO DATA FOR SENDING...\r\n", 0, true);

                scan_result.read_marker = temp_marker;
            } else if (error) {
            } else {
                commited                = 1;
                scan_result.read_marker = temp_marker;
            }

            // display read marker
            if (!error) {
                terminal_buffer_lock();
                sprintf(alert_str, "read ptr after: 0x%x %d cnt=%d\r\n", scan_result.read_marker.flash_address,
                        scan_result.read_marker.event_id, event_count);
                DisplayMessage(alert_str, 0, false);
                terminal_buffer_release();
            }

            xSemaphoreGive(event_semaphore);

            if (data_found) {
                if (commited) {
                    DisplayMessage("Event Data Transfer - Commit\r\n", 0, true);

                    if (last_valid_marker.event_id > 0) {

                        // SetCounter(&last_valid_marker.event_id, COUNTER_LAST_READ_EVENT_ID);
                        // SetCounter(&last_valid_marker.flash_address, COUNTER_LAST_READ_ADDRESS);
                        flash_counter_write(FLASH_COUNTER_IDX_LAST_SENT_EVENT_ID, (uint8_t *)&last_valid_marker.event_id, 4);
                        flash_counter_write(FLASH_COUNTER_IDX_LAST_SENT_EVENT_ADDRESS, (uint8_t *)&last_valid_marker.flash_address, 4);

                        terminal_buffer_lock();
                        sprintf(alert_str, "last sent event id: %d\r\n", last_valid_marker.event_id);
                        DisplayMessage(alert_str, 0, false);
                        terminal_buffer_release();
                    }
                }
            }
        } else {
            DisplayMessage("Event Data Transfer - Roll back\r\n", 0, true);
            CreateDebugEvent(EVENT_DEBUG_ROLLBACK_REASONS, error, false);
        }

        vTaskDelay(3000);
    }

    if (!end_session_sent) {
        tx_msg_len = Build_R2_Packet(COMMAND_SEND_CRC, 0, transfer_buffer + 6, 0, crc, 15);
        transfer_buffer[0] = 0x80;
        transfer_buffer[1] = tx_msg_len + 4;
        memcpy(transfer_buffer + 2, &packet_count, 4);
        send_data(transfer_buffer, tx_msg_len + 6);
    }

    // CreateLastSentEvent(3);

    ///////////////

    if (scan_result.missing_event_id > 0) {
        CreateDebugEvent(EVENT_DEBUG_MISSING_EVENT_ID, scan_result.missing_event_id, true);
    }

    return error;
}

uint16_t Build_R2_Packet(uint8_t cmd, uint8_t flags, uint8_t *sending_buffer, uint32_t value, uint16_t crc, uint8_t device_len)
{
    uint32_t timStmp  = 0;
    uint8_t *ptr      = NULL;
    uint16_t R2BufLen = (device_len + 18);

    flags &= 0xF0;

    // Preamble
    sending_buffer[0] = 'D';
    sending_buffer[1] = '2';

    // message length
    sending_buffer[2] = ((uint16_t)(R2BufLen & 0xFF));
    sending_buffer[3] = ((uint16_t)(R2BufLen & 0xFF00) >> 8);

    // time stamp
    // timStmp = RTC_Calc_TimeStamp();
    ptr = (uint8_t *)&timStmp;

    sending_buffer[4] = (uint8_t)((*ptr++));
    sending_buffer[5] = (uint8_t)((*ptr++));
    sending_buffer[6] = (uint8_t)((*ptr++));
    sending_buffer[7] = (uint8_t)((*ptr));

    // conn/disc status //

    /*
    switch (cmd)
    {
    case GSM_COMMAND_CONNECT_SEND_EVENTS:
            sending_buffer[8] = 0x00;
            break;

    case GSM_COMMAND_SEND_CRC_KEEP_ALIVE:
            sending_buffer[8] = 0x01;
            break;
    }
    */

    sending_buffer[8] = flags | cmd;

    // event counter //

    ptr = (uint8_t *)&value;

    sending_buffer[9]  = (uint8_t)((*ptr++));
    sending_buffer[10] = (uint8_t)((*ptr++));
    sending_buffer[11] = (uint8_t)((*ptr++));
    sending_buffer[12] = (uint8_t)((*ptr));

    // device ID type //

    sending_buffer[13] = '\0';
    sending_buffer[14] = '\0';

    // memcpy(&sending_buffer[15], dev_params.dpDvcID, 15);
    memcpy(&sending_buffer[15], device_config.DeviceID, device_len);
    sending_buffer[R2BufLen - 3] = '\0';

    // CRC
    // crcClc = CRC16_Calc(sending_buffer, R2BufLen-2, 0);

    sending_buffer[R2BufLen - 2] = (uint8_t)((crc & 0XFF));
    sending_buffer[R2BufLen - 1] = (uint8_t)((crc & 0XFF00) >> 8);

    return R2BufLen;
}

bool send_data(uint8_t *data, uint8_t len)
{
    bool ret_ble_error;

    if (driver_behaviour_state.print_sent_event_data_mode) {
        terminal_buffer_lock();
        DisplayMessage("\r\n", 2, false);
        ConvertDataToHexString(data, alert_str, len / 3);
        DisplayMessage(alert_str, 0, false);
        DisplayMessage("\r\n", 2, false);
        terminal_buffer_release();
    }

    ret_ble_error = !ble_services_notify_event_transfer(transfer_buffer, len);
}