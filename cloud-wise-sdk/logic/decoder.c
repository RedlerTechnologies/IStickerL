#include "decoder.h"

#include "FreeRTOS.h"
#include "ble_file_transfer.h"
#include "drivers/buzzer.h"
#include "drivers/flash.h"
#include "events.h"
#include "flash_data.h"
#include "hal/hal_boards.h"
#include "logic/serial_comm.h"
#include "monitor.h"
#include "nrf_delay.h"
#include "recording.h"
#include "semphr.h"
#include "timers.h"
#include "tracking_algorithm.h"

#include <string.h>

#define NRF_LOG_MODULE_NAME decoder
#define NRF_LOG_LEVEL CLOUD_WISE_DEFAULT_LOG_LEVEL
#include "nrfx_log.h"
NRF_LOG_MODULE_REGISTER();

#define MIN_BUFFER_SIZE 64

#define MIN_VALID_SECONDS 600000000

extern xSemaphoreHandle     event_semaphore;
extern DriverBehaviourState driver_behaviour_state;

char Flash_buffer[MAX_FLASH_BUFFER + 4];

ScanResult scan_result;

unsigned GetTimeFromEvent(char *buffer);
void     PrintEvent(unsigned char *buffer, unsigned address, unsigned char error);

void InitMarker(FlashMarker *marker, unsigned char *buffer, unsigned flash_address, unsigned short buffer_size, unsigned short offset,
                unsigned char options)
{
    marker->buffer        = buffer;
    marker->buffer_size   = buffer_size;
    marker->offset        = offset;
    marker->flash_address = flash_address;
    marker->options       = options;
    marker->event_id      = 0;
    marker->empty         = 0;
    marker->limit_address = 0;

    memset(buffer, 0x00, buffer_size);
}

void ReadEventsToMarker(FlashMarker *marker)
{
    unsigned       search_event_id = 0;
    unsigned       flash_address, last_valid_address, start_sector_address;
    unsigned short crc, calc_crc;
    unsigned short i, j, k, len, offset, buffer_size;

    unsigned char *ptr;
    unsigned char *f_buffer;

    unsigned short loop_count = 0;
    unsigned short read_bytes = MAX_FLASH_BUFFER;
    unsigned short sector_id;
    unsigned char  completed = 0;
    unsigned char  error     = 0;
    unsigned char  found     = 0;
    unsigned char  crc_error;
    unsigned char  ignore_crc_error;

    unsigned char only_event_id = 0;
    unsigned char options       = marker->options;
    buffer_size                 = marker->buffer_size;

    search_event_id = marker->event_id;
    flash_address   = marker->flash_address;

    sector_id            = GetSectorID(flash_address);
    start_sector_address = GetFlashAddress(sector_id);

    ignore_crc_error = (options & MARKER_OPT_IGNORE_CRC_ERROR) > 0;

    i                    = 0;
    marker->event_id     = 0;
    marker->max_event_id = 0;
    marker->event_count  = 0;
    offset               = marker->offset;
    last_valid_address   = 0;
    j                    = flash_address % FLASH_SECTOR_SIZE;

    if (options & MARKER_OPT_FIND_ID)
        options |= (MARKER_OPT_GET_LAST);

    if (options & MARKER_OPT_EVENT_ID_ONLY) {
        only_event_id = 1;
        read_bytes    = MIN_BUFFER_SIZE;
    }

    if (options & MARKER_OPT_GET_FIRST) {
        read_bytes = MIN_BUFFER_SIZE;
    }

    while (!completed) {
        // SetMonitor(TRACKING_TASK_ID);

        // FillReadAddress(wrBuff, start_sector_address + j);

        if (flash_read_buffer(Flash_buffer, start_sector_address + j, MAX_FLASH_BUFFER)) {
            i        = 0;
            f_buffer = Flash_buffer + 4;
        } else {
            error = MARKER_ERROR_BUSY;
        }

        /*
                if (xSemaphoreTake(semFlsIO, (portTickType)SEM_FLS_IO_TIMOUT) == pdTRUE) {
                    i = 0;
                    memset(Flash_buffer, 0x00, MAX_FLASH_BUFFER);

                    if (Flash_Read_Data(wrBuff, 4, Flash_buffer, read_bytes)) {
                        xSemaphoreGive(semFlsIO);
                    } else {
                        error = MARKER_ERROR_READ_FLASH_FAILED;
                    }
                } else {
                    // if (scan_result != NULL) scan_result->read_error_sectors++;
                    error = MARKER_ERROR_BUSY;
                }
                */

        if (error)
            break;

        while (!completed) {

            // patch: avoid infinite loop
            loop_count++;
            if (loop_count >= FLASH_SECTOR_SIZE) {
                error     = 1;
                completed = 1;
                break;
            }

            error = 0;

            if (j == 0) {
                if (f_buffer[0] == 'D') {
                    // ..
                } else if (f_buffer[0] != 0xFF) {
                    error     = MARKER_ERROR_PREMABLE;
                    completed = 1;
                } else {
                    error     = MARKER_ERROR_EMPTY_SECTOR;
                    completed = 1;
                }

                if (error)
                    break;
            }

            ptr       = &f_buffer[i];
            crc_error = 0;

            if (offset + 14 > buffer_size) {
                completed = 1;
                break;
            }

            if (i + 14 > read_bytes) {
                break;
            }

            if (j + 14 > FLASH_SECTOR_SIZE) {
                completed = 1;
                break;
            }

            if ((options & MARKER_OPT_GET_LAST) && ptr[0] == 0xFF) {
                completed = 1;
                break;
            }

            if (ptr[0] != 'D') {
                error = MARKER_ERROR_PREMABLE;
            }

            if (!error) {
                if (ptr[1] != '0' && ptr[1] != '1')
                    error = MARKER_ERROR_PREMABLE;
            }

            if (!error) {
                len = (ptr[3] << 8) + ptr[2];

                if (len < 14)
                    error = MARKER_ERROR_LENGTH;
                else if (len >= MIN_BUFFER_SIZE)
                    error = MARKER_ERROR_LENGTH;
                else {
                    if (offset + len > buffer_size) {
                        completed = 1;
                        break;
                    }

                    if (i + len > read_bytes) {
                        break;
                    }

                    if (j + len > FLASH_SECTOR_SIZE) {
                        completed = 1;
                        break;
                    }
                }
            }

            if (!error) {
                calc_crc = CRC16_Calc(ptr, (len - 2), 0);
                crc      = (ptr[len - 1] << 8) + ptr[len - 2];

                if (crc != calc_crc) {
                    crc_error = 1;
                    if (!ignore_crc_error)
                        error = MARKER_ERROR_CRC;
                }
            }

            if (error) {
                if (options & MARKER_OPT_GET_FIRST) {
                    completed = 1;
                    break;
                }

                // if error is identified. it's no use to continue
                // checking for error in this sector.
                i++;
                j++;
                k++;

                if (k > MIN_BUFFER_SIZE) {
                    completed = 1;
                    error     = MARKER_ERROR_SYNC;
                    break;
                }

                continue;
            } else {
                k                  = 0;
                marker->event_id   = (ptr[11] << 16) + (ptr[10] << 8) + ptr[9];
                marker->event_size = len;
                last_valid_address = (start_sector_address + j + len);
                marker->event_count++;

                if (scan_result.missing_event_id == 0 && scan_result.previous_event_id > 0) {
                    if (marker->event_id >= scan_result.previous_event_id) {
                        if (marker->event_id != (scan_result.previous_event_id + 1))
                            scan_result.missing_event_id = (scan_result.previous_event_id + 1);
                    }
                }
                scan_result.previous_event_id = marker->event_id;

                if (marker->event_id > marker->max_event_id)
                    marker->max_event_id = marker->event_id;

                if (options & MARKER_OPT_PRINT) {
                    PrintEvent(ptr, (start_sector_address + j), crc_error);
                } else if (options & MARKER_OPT_PRINT_ID) {
                    terminal_buffer_lock();
                    sprintf(alert_str, (const char *)"EID: %d\r\n", marker->event_id);
                    DisplayMessage(alert_str, 0, false);
                    terminal_buffer_release();
                }

                if (!crc_error || ignore_crc_error)
                    memcpy(marker->buffer + offset, ptr, len);

                if (options & MARKER_OPT_FIND_ID) {
                    if (search_event_id == marker->event_id) {
                        found     = 1;
                        completed = 1;
                        break;
                    }
                } else
                    found = 1;

                i += len;
                j += len;

                if (options & MARKER_OPT_GET_CONTINUE) {
                    if (!crc_error || (options & ignore_crc_error))
                        offset += len;
                }

                if (last_valid_address == marker->limit_address) {
                    completed = 1;
                    break;
                }

                if (only_event_id || (options & MARKER_OPT_GET_FIRST)) {
                    completed = 1;
                    break;
                }
            }

            if ((options & (MARKER_OPT_GET_LAST | MARKER_OPT_GET_CONTINUE)) == 0 && found) {
                completed = 1;
                break;
            }

            error = 0;
        }
    }

    if (last_valid_address > 0) {
        CheckFlashAddressLimit(&last_valid_address);
        marker->flash_address = last_valid_address;
    }

    if (!error && !found)
        error = MARKER_ERROR_NOT_FOUND;

    marker->offset = offset;
    marker->error  = error;
}

bool InitWriteMarker(ScanResult *scan_result)
{
    FlashMarker    marker;
    unsigned       time = 0;
    unsigned       flash_address;
    unsigned short sector_id;
    unsigned short max_sector_id;

    unsigned char options;

    scan_result->first_marker.event_id    = 0;
    scan_result->max_marker.flash_address = 0;
    scan_result->max_marker.event_id      = 0;
    scan_result->min_event_id             = 0;

    memset((unsigned char *)(&scan_result->time_marker), 0x00, sizeof(xTimeMarker));
    scan_result->time_logged_in_flash = 0;

    flash_address = FLASH_EVENTS_START_ADDRESS;

    options = (MARKER_OPT_GET_FIRST);

    for (sector_id = 0; sector_id < NUM_DATA_SECTORS; sector_id++) {
        // SetMonitor(TRACKING_TASK_ID);
        // InitMarker(&marker, scan_buffer, flash_address, SCAN_BUFFER_SIZE, 0, options);
        InitMarker(&marker, Flash_buffer, flash_address, MAX_FLASH_BUFFER, 0, options);

        ReadEventsToMarker(&marker);

        if (marker.error == 0 && !marker.empty) {
            if (marker.event_id >= scan_result->max_marker.event_id) {
                scan_result->max_marker.event_id      = marker.event_id;
                scan_result->max_marker.flash_address = flash_address;
                max_sector_id                         = sector_id;
            }

            if (scan_result->first_marker.event_id == 0) {
                scan_result->min_event_id               = marker.event_id;
                scan_result->first_marker.event_id      = marker.event_id;
                scan_result->first_marker.flash_address = marker.flash_address;
            }

            if (marker.event_id < scan_result->min_event_id) {
                scan_result->min_event_id = marker.event_id;
            }
        }

        flash_address += FLASH_SECTOR_SIZE;
    }

    // find the max inside sector

    if (scan_result->max_marker.flash_address) {
        options = (MARKER_OPT_GET_LAST);

        // InitMarker(&marker, scan_buffer, scan_result->max_marker.flash_address, SCAN_BUFFER_SIZE, 0, options);
        InitMarker(&marker, Flash_buffer, scan_result->max_marker.flash_address, MAX_FLASH_BUFFER, 0, options);

        ReadEventsToMarker(&marker);

        if ((marker.error == 0) && (marker.event_id >= scan_result->max_marker.event_id)) {
            scan_result->max_marker.flash_address = marker.flash_address;
            scan_result->max_marker.event_id      = marker.event_id;
        } else {
            // some error occure finding the maximum event in the @maximum@ sector
            // better to write the next event in the next sector

            max_sector_id++;
            CheckSectorLimit(&max_sector_id);

            scan_result->max_marker.flash_address = GetFlashAddress(max_sector_id);
            scan_result->max_marker.event_id += 1000;
        }
    }

    // estimate flash "working time"

    if (scan_result->max_marker.flash_address) {
        options   = (MARKER_OPT_GET_FIRST);
        sector_id = GetSectorID(scan_result->max_marker.flash_address);

        while (sector_id > 0) {
            flash_address = GetFlashAddress(sector_id);
            // SetMonitor(TRACKING_TASK_ID);
            // InitMarker(&marker, scan_buffer, flash_address, SCAN_BUFFER_SIZE, 0, options);
            InitMarker(&marker, Flash_buffer, flash_address, MAX_FLASH_BUFFER, 0, options);

            ReadEventsToMarker(&marker);

            // time = GetTimeFromEvent(scan_buffer);
            time = GetTimeFromEvent(Flash_buffer);

            if (time > 0) {
                if (scan_result->time_marker.time == 0) {
                    scan_result->time_marker.time      = time;
                    scan_result->time_marker.sector_id = sector_id;
                } else if (scan_result->time_logged_in_flash == 0) {
                    time = scan_result->time_marker.time - time;

                    scan_result->time_logged_in_flash =
                        (float)time * (NUM_DATA_SECTORS) / (scan_result->time_marker.sector_id - sector_id) / 3600;
                    break;
                }
            }

            sector_id--;
        }
    }

    // if error, start from start

    scan_result->write_marker.flash_address = FLASH_EVENTS_START_ADDRESS;
    scan_result->write_marker.event_id      = 1;

    if (scan_result->max_marker.event_id > 0) {
        scan_result->write_marker.event_id      = scan_result->max_marker.event_id + 1;
        scan_result->write_marker.flash_address = scan_result->max_marker.flash_address;

        return true;
    }

    return false;
}

// the marker parameter contains the address of the
bool InitReadMarker(ScanResult *scan_result, unsigned int search_event_id)
{
    FlashMarker marker;

    unsigned       flash_address;
    unsigned short start_sector_id;
    unsigned short end_sector_id;
    unsigned short sector_id;
    unsigned char  found = 0;
    bool           status;

    unsigned char options;

    terminal_buffer_lock();
    sprintf(alert_str, (const char *)"Search Event ID: %d\r\n", search_event_id);
    DisplayMessage(alert_str, 0, false);
    terminal_buffer_release();

    if (search_event_id >= scan_result->first_marker.event_id && search_event_id <= scan_result->max_marker.event_id) {
        flash_address   = scan_result->first_marker.flash_address;
        start_sector_id = GetSectorID(flash_address);

        flash_address = scan_result->max_marker.flash_address;
        end_sector_id = GetSectorID(flash_address);
    } else {
        flash_address   = scan_result->max_marker.flash_address;
        start_sector_id = GetSectorID(flash_address);
        start_sector_id++;
        end_sector_id = GetSectorID(FLASH_EVENTS_END_ADDRESS) - 1;
    }

    options = (MARKER_OPT_EVENT_ID_ONLY);

    for (sector_id = start_sector_id; sector_id <= end_sector_id; sector_id++) {
        flash_address = GetFlashAddress(sector_id);

        // SetMonitor(TRACKING_TASK_ID);
        // InitMarker(&marker, scan_buffer, flash_address, SCAN_BUFFER_SIZE, 0, options);
        InitMarker(&marker, Flash_buffer, flash_address, MAX_FLASH_BUFFER, 0, options);

        ReadEventsToMarker(&marker);

        if (marker.error == 0 && !marker.empty) {
            if (search_event_id < marker.event_id) {
                break;
            }
        }
    }

    while (sector_id > 0) {
        sector_id--;

        flash_address = GetFlashAddress(sector_id);

        // SetMonitor(TRACKING_TASK_ID);
        // InitMarker(&marker, scan_buffer, flash_address, SCAN_BUFFER_SIZE, 0, options);
        InitMarker(&marker, Flash_buffer, flash_address, MAX_FLASH_BUFFER, 0, options);

        ReadEventsToMarker(&marker);

        if (marker.error == 0 && !marker.empty) {
            if (search_event_id >= marker.event_id) {
                scan_result->read_marker.flash_address = flash_address;
                scan_result->read_marker.event_id      = marker.event_id;
                found                                  = 1;
                break;
            }
        }
    }

    if (found) {
        options = (MARKER_OPT_FIND_ID);

        // InitMarker(&marker, scan_buffer, flash_address, SCAN_BUFFER_SIZE, 0, options);
        InitMarker(&marker, Flash_buffer, flash_address, MAX_FLASH_BUFFER, 0, options);
        marker.event_id = search_event_id;

        ReadEventsToMarker(&marker);

        if (marker.error == 0) {
            if (search_event_id == marker.event_id) {
                scan_result->read_marker.flash_address = marker.flash_address - marker.event_size;
                scan_result->read_marker.event_id      = marker.event_id;
            } else {
                found = 0;
            }
        } else {
            options = (MARKER_OPT_GET_FIRST);
            found   = 0;

            while (sector_id > 0) {
                flash_address = GetFlashAddress(sector_id);
                // InitMarker(&marker, scan_buffer, flash_address, SCAN_BUFFER_SIZE, 0, options);
                InitMarker(&marker, Flash_buffer, flash_address, MAX_FLASH_BUFFER, 0, options);

                ReadEventsToMarker(&marker);

                if (marker.error == 0) {
                    scan_result->read_marker.flash_address = flash_address;
                    scan_result->read_marker.event_id      = marker.event_id;
                    found                                  = 1;
                    break;
                }

                sector_id--;
            }
        }
    }

    if (found)
        status = true;
    else
        status = false;

    return status;
}

unsigned GetTimeFromEvent(char *buffer)
{
    unsigned time = 0;

    memcpy((unsigned char *)(&time), buffer + 4, 4);

    if (time < MIN_VALID_SECONDS)
        time = 0;

    return time;
}

unsigned short ScanRecords(unsigned int search_event_id)
{
    FlashMarker    marker;
    unsigned short action_code    = 0;
    unsigned char  event_id_valid = 0;
    unsigned char  research       = 0;
    bool           status;
    bool           save_rx_ptr = false;

    if (search_event_id > 0)
        save_rx_ptr = true;

    // find write pointer
    status = InitWriteMarker(&scan_result);

    // search_event_id = 1; // for debugging only

    if (!status)
        action_code |= FLASH_SCAN_WRITE_PTR_RESTARTED;

    // load saved read pointer from flash
search_event_id_label:

    if (search_event_id > 0) {
        if (search_event_id < scan_result.min_event_id)
            search_event_id = scan_result.min_event_id;

        if (search_event_id > scan_result.max_marker.event_id) {
            status = false;
        } else {
            status = InitReadMarker(&scan_result, search_event_id);
            if (status)
                action_code |= FLASH_SCAN_READ_PTR_CHANGED_OK;
        }
    } else {
        // status = GetCounter(&scan_result.read_marker.event_id, COUNTER_LAST_READ_EVENT_ID);
        flash_counter_read(FLASH_COUNTER_IDX_LAST_SENT_EVENT_ID, (uint8_t *)&scan_result.read_marker.event_id, 4);
        status = (scan_result.read_marker.event_id != 0xFFFFFFFF);

        if (status) {
            event_id_valid = 1;
            // status         = GetCounter(&scan_result.read_marker.flash_address, COUNTER_LAST_READ_ADDRESS);
            flash_counter_read(FLASH_COUNTER_IDX_LAST_SENT_EVENT_ADDRESS, (uint8_t *)&scan_result.read_marker.flash_address, 4);
            status = (scan_result.read_marker.event_id != 0xFFFFFFFF);

            if (status) {
                // InitMarker(&marker, scan_buffer, scan_result.read_marker.flash_address, SCAN_BUFFER_SIZE, 0,
                // (MARKER_OPT_IGNORE_CRC_ERROR));
                InitMarker(&marker, Flash_buffer, scan_result.read_marker.flash_address, MAX_FLASH_BUFFER, 0,
                           (MARKER_OPT_IGNORE_CRC_ERROR));

                ReadEventsToMarker(&marker);

                if (marker.event_id != scan_result.read_marker.event_id)
                    status = false;
            }
        }

        if (!status)
            action_code |= FLASH_SCAN_READ_PTR_RESTORED_FAILED;
    }

    if (!status && event_id_valid && !research) {
        search_event_id = scan_result.read_marker.event_id;
        research        = 1;

        goto search_event_id_label;
    }

    // last try to set the read pointer.   Read pointer equal the write pointer
    if (!status) {
        scan_result.read_marker.flash_address = scan_result.write_marker.flash_address;
        scan_result.read_marker.event_id      = scan_result.write_marker.event_id;
        action_code |= FLASH_SCAN_READ_SET_TO_WRITE_PTR;
        save_rx_ptr = true;
    }

    if (save_rx_ptr) {
        flash_counter_write(FLASH_COUNTER_IDX_LAST_SENT_EVENT_ID, (uint8_t *)&scan_result.read_marker.event_id, 4);
        flash_counter_write(FLASH_COUNTER_IDX_LAST_SENT_EVENT_ADDRESS, (uint8_t *)&scan_result.read_marker.flash_address, 4);
    }

    return action_code;
}

unsigned short InitEventFlashStructure(unsigned int search_event_id)
{
    unsigned short action_code = 0;

    DisplayMessage("\r\nScanning Event Flash...\r\n", 0, true);

    action_code = ScanRecords(search_event_id);

    // display to terminal //

    terminal_buffer_lock();

    sprintf(alert_str, "\r\nread ptr: eid=%d address=0x%x\r\n", scan_result.read_marker.event_id, scan_result.read_marker.flash_address);
    DisplayMessage(alert_str, 0, false);

    sprintf(alert_str, "\r\nwrite ptr: eid=%d address=0x%x\r\n", scan_result.write_marker.event_id, scan_result.write_marker.flash_address);
    DisplayMessage(alert_str, 0, false);

    terminal_buffer_release();

    return action_code;
}

void PrintEvent(unsigned char *buffer, unsigned address, unsigned char error)
{
    unsigned       value;
    unsigned       event_id;
    unsigned       time;
    unsigned short sector_num;
    unsigned short event_type;
    unsigned short record_len;
    unsigned short i = 0;
    unsigned char  tmp;

    sector_num = (address - FLASH_EVENTS_START_ADDRESS) / FLASH_SECTOR_SIZE;

    memcpy((unsigned char *)&value, buffer + 9, 4);
    event_id = value;

    memcpy((unsigned char *)&value, buffer + 4, 4);
    time = value;

    memcpy((unsigned char *)&value, buffer + 13, 2);
    event_type = value;

    memcpy((unsigned char *)&value, buffer + 2, 2);
    record_len = value;

    terminal_buffer_lock();
    memset(alert_str, 0x00, ALERT_BUFFER_SIZE);
    sprintf(alert_str, "\r\n%d,%d,%d,0x%x,%d,%d,%d,", event_id, event_type, time, address, sector_num, record_len, error);
    DisplayMessage(alert_str, 0, false);

    memset(alert_str, 0x00, ALERT_BUFFER_SIZE);

    if (driver_behaviour_state.print_event_data) {

        ConvertDataToHexString(buffer, alert_str, record_len);
        DisplayMessage(alert_str, 0, false);
    }

    terminal_buffer_release();
}

void PrintAllEventData(void)
{
    FlashMarker    marker;
    unsigned short sector_id = 0;
    unsigned short msg_len;

    scan_result.last_printed_event_id = 0;

    if (xSemaphoreTake(event_semaphore, 500))
        marker.flash_address = FLASH_EVENTS_START_ADDRESS;

    DisplayMessage("\r\n--- Print All Events ---\r\n", 0, true);

    while (1) {
        monitor_task_set_all();

        InitMarker(&marker, Flash_buffer, marker.flash_address, MAX_FLASH_BUFFER, 0,
                   (MARKER_OPT_GET_CONTINUE | MARKER_OPT_PRINT | MARKER_OPT_IGNORE_CRC_ERROR));

        ReadEventsToMarker(&marker);

        terminal_buffer_lock();

        memset(alert_str, 0x00, ALERT_BUFFER_SIZE);

        if ((marker.flash_address % FLASH_SECTOR_SIZE) > (FLASH_SECTOR_SIZE - 64)) {
            sprintf(alert_str, "\r\nsector %d - OK\r\n", sector_id);
        } else if ((marker.error > 0) && (marker.error != MARKER_ERROR_EMPTY_SECTOR)) {
            sprintf(alert_str, "\r\nsector %d - contains error\r\n", sector_id);
        } else if (marker.error == (MARKER_ERROR_EMPTY_SECTOR)) {
            sprintf(alert_str, "\r\nsector %d - empty\r\n", sector_id);
        }

        msg_len = strlen(alert_str);

        if (msg_len > 0) {
            DisplayMessage(alert_str, 0, false);

            sector_id = GetSectorID(marker.flash_address);
            sector_id++;

            marker.flash_address = GetFlashAddress(sector_id);
        }

        terminal_buffer_release();

        if (marker.flash_address >= FLASH_EVENTS_END_ADDRESS || marker.flash_address <= FLASH_EVENTS_START_ADDRESS) {
            break;
        }
    }

    xSemaphoreGive(event_semaphore);
}

void ConvertDataToHexString(uint8_t *data, uint8_t *converted_data, uint8_t len)
{
    uint8_t i;
    uint8_t tmp;

    for (i = 0; i < len; i++) {
        tmp = ((data[i] >> 4) & 0x0F);
        tmp += (tmp < 10) ? '0' : ('A' - 10);
        converted_data[2 * i] = tmp;

        tmp = (data[i] & 0x0F);
        tmp += (tmp < 10) ? '0' : ('A' - 10);
        converted_data[2 * i + 1] = tmp;
    }
}