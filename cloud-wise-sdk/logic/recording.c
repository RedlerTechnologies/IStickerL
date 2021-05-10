#include "recording.h"

#include "FreeRTOS.h"
#include "ble/ble_services_manager.h"
#include "ble_file_transfer.h"
#include "drivers/buzzer.h"
#include "drivers/flash.h"
#include "events.h"
#include "flash_data.h"
#include "hal/hal_boards.h"
#include "logic/serial_comm.h"
#include "monitor.h"
#include "nrf_delay.h"
#include "semphr.h"
#include "timers.h"
#include "tracking_algorithm.h"

#include <stdlib.h>
#include <string.h>
#include <time.h>

static uint8_t              flash_buffer[300];
extern xSemaphoreHandle     tx_uart_semaphore;
extern DriverBehaviourState driver_behaviour_state;

#define NRF_LOG_MODULE_NAME recording
#define NRF_LOG_LEVEL CLOUD_WISE_DEFAULT_LOG_LEVEL
#include "nrfx_log.h"
NRF_LOG_MODULE_REGISTER();

static void record_create_new(void);
static void SendRecordAlert(uint32_t record_id);

AccRecord acc_record;

void record_init(void)
{
    uint8_t  buffer[8] = {'\0'};
    int32_t  value;
    uint32_t min_record_id = 0xFFFFFFFF;
    uint32_t flash_address;
    int32_t  min_id = -1;
    uint16_t i;

    bool flag;

    // build record structure
    // here
    // ..

    for (i = 0; i < MAX_RECORDS; i++) {
        flash_address = FLASH_RECORDS_START_ADDRESS + i * RECORD_SIZE;
        flag          = flash_read_buffer(buffer, flash_address, 4);

        if (flag) {
            memcpy((uint8_t *)&value, buffer + 4, 4);

            if (value == -1) {
                min_id = (i - 1);
                break;
            } else if (value < min_record_id) {
                min_record_id = value;
                min_id        = (i - 1);
            }
        } else {
        }
    }

    // point to the oldest record date here
    acc_record.record_num = min_id;

    record_create_new();
}

static void record_create_new(void)
{
    uint32_t flash_address;
    uint8_t  i;
    uint8_t  ratio;

    memset(acc_record.samples1, 0x0, RECORD_BUFFER_SAMPLE_SIZE * 6);

    acc_record.sample_index  = 0;
    acc_record.flash_address = 0;
    acc_record.state         = ACC_RECORD_START;
    acc_record.buffer_stage  = 0;
    acc_record.record_id     = 0;
    acc_record.file_crc      = 0;

    acc_record.record_num++;
    if (acc_record.record_num >= MAX_RECORDS)
        acc_record.record_num = 0;

    // delete region (sector) here

    flash_address = FLASH_RECORDS_START_ADDRESS + acc_record.record_num * RECORD_SIZE;
    ratio         = (RECORD_SIZE / FLASH_SECTOR_SIZE);

    for (i = 0; i < ratio; i++) {
        flash_erase_sector(flash_address);
        flash_address += FLASH_SECTOR_SIZE;
    }
}

uint8_t record_scan_for_new_records(bool forced)
{
    static uint8_t read_buffer[12];
    uint8_t *      buffer;
    uint32_t       flash_address;
    uint32_t       record_id;
    int16_t        index = -1;
    uint32_t       duration;
    uint16_t       max_record_id = 0;
    uint8_t        i;
    uint8_t        pending_counter = 0;
    bool           flag;

    duration = timeDiff(xTaskGetTickCount(), acc_record.last_found_record_time) / 1000;

    if ((duration > 30 && ble_services_is_connected()) || forced) {
        acc_record.last_found_record_time = xTaskGetTickCount();

        for (i = 0; i < MAX_RECORDS; i++) {
            flash_address = FLASH_RECORDS_START_ADDRESS + (i + 1) * RECORD_SIZE - RECORD_TERMINATOR_SIZE;

            flag = flash_read_buffer(read_buffer, flash_address, 8);
            if (flag) {
                buffer = read_buffer + 4;
                // if record is completed but not sent
                if (buffer[0] == 0 && buffer[1] == 0xFF) {
                    pending_counter++;
                    flash_address = FLASH_RECORDS_START_ADDRESS + i * RECORD_SIZE;

                    flag = flash_read_buffer(read_buffer, flash_address, 4);

                    if (flag) {
                        memcpy((uint8_t *)&record_id, read_buffer + 4, 4);

                        if (record_id > max_record_id) {
                            index         = i;
                            max_record_id = record_id;
                        }
                    }
                }
            }
        }
    }
    if (index >= 0 && ble_services_is_connected()) {
        // SendRecordBleAlert(record_id);
        SendRecordAlert(record_id);
        // ??????????? DelaySleep(180, 0);
    }

    if (!forced) {
        terminal_buffer_lock();
        sprintf(alert_str, "\r\nRecords #%d\r\n", pending_counter);
        DisplayMessage(alert_str, 0, false);
        terminal_buffer_release();
    }

    return pending_counter;
}

int16_t record_search(uint32_t record_id)
{
    uint8_t  buffer[8];
    uint32_t flash_address;
    int32_t  value;
    int16_t  index = -1;
    uint8_t  i;

    for (i = 0; i < MAX_RECORDS; i++) {
        flash_address = FLASH_RECORDS_START_ADDRESS + i * RECORD_SIZE;

        // SetMonitorAll();

        flash_read_buffer(buffer, flash_address, 4);
        memcpy((uint8_t *)&value, buffer + 4, 4);

        if (value == record_id) {
            index = i;
            break;
        }
    }

    return index;
}

void record_trigger(uint8_t reason)
{
    static uint32_t record_id = 0;

    // ????????? DelaySleep(30, 0);

    acc_record.state         = ACC_RECORD_IDENTIFIED;
    acc_record.record_reason = reason;

    // ????????? acc_record.record_id     = scan_result.write_marker.event_id;
    record_id            = GetRandomNumber();
    acc_record.record_id = record_id;
}

bool record_is_active(void)
{
    bool res;

    res = (acc_record.state != ACC_RECORD_START);

    return res;
}

static void get_header(uint8_t *header)
{
    Calendar c;
    GetSystemTime(&c);
    uint32_t *ptr;
    uint8_t   flags          = 0;
    uint8_t   param_enabled1 = 0;

    memset(header, 0x00, RECORD_HEADER_SIZE);

    header[4] = c.hour;
    header[5] = c.minute;
    header[6] = c.seconds;
    header[7] = c.day;
    header[8] = c.month;
    header[9] = c.year;

    header[13] = RECORD_SAMPLE_FREQ_50HZ;
    header[14] = acc_record.record_reason;

    param_enabled1 = (REACORD_PARAM_ACC_X | REACORD_PARAM_ACC_Y | REACORD_PARAM_ACC_Z);
    header[10]     = param_enabled1;

    ptr = &acc_record.record_id;
    memcpy(header, ptr, 4);

    /*
        if (driver_behavior_state.calibration_state == CALIRATION_STATE_DONE) {
            header[16] = (signed char)driver_behavior_state.angle1;
            header[17] = (signed char)driver_behavior_state.angle2;
            flags |= REACORD_FLAGS_CALIBRATED;
        }
        */

    flags |= REACORD_FLAGS_CALIBRATED;

    header[12] = flags;
    header[15] = RECORD_RESOLUTION;
}

void record_write_status(uint8_t record_num, uint8_t indication_idx, uint8_t value)
{
    uint8_t buffer[8];
    bool    status;

    uint32_t flash_address;

    flash_address = FLASH_RECORDS_START_ADDRESS + (record_num + 1) * RECORD_SIZE - RECORD_TERMINATOR_SIZE;
    flash_address += indication_idx;

    // status = Program_Flash((unsigned char *)(&value), 1, flash_address);
    memcpy(buffer + 4, (uint8_t *)&value, 1);
    status = flash_write_buffer(buffer, flash_address, 1);
}

void record_add_sample(AccConvertedSample *acc_sample)
{
    uint8_t *ptr;
    uint8_t *ptr2;
    uint32_t flash_address;
    int16_t  sample_value;
    uint16_t sample_index;
    uint16_t len;
    uint8_t  end_of_buffer = 0;
    bool     save          = false;

    if (acc_record.buffer_stage) {
        ptr  = (unsigned char *)acc_record.samples2;
        ptr2 = (unsigned char *)acc_record.samples1;
    } else {
        ptr  = (unsigned char *)acc_record.samples1;
        ptr2 = (unsigned char *)acc_record.samples2;
    }

    sample_index = acc_record.sample_index * 6;

    sample_value = (short)(acc_sample->drive_direction);
    memcpy(ptr + sample_index, (unsigned char *)(&sample_value), 2);
    sample_index += 2;

    sample_value = (short)(acc_sample->turn_direction);
    memcpy(ptr + sample_index, (unsigned char *)(&sample_value), 2);
    sample_index += 2;

    sample_value = (short)(acc_sample->earth_direction);
    memcpy(ptr + sample_index, (unsigned char *)(&sample_value), 2);

    acc_record.sample_index++;

    if (acc_record.sample_index >= RECORD_BUFFER_SAMPLE_SIZE)
        end_of_buffer = 1;

    switch (acc_record.state) {
    case ACC_RECORD_START:
        break;

    case ACC_RECORD_IDENTIFIED:
        // copy buffer sample 1 to sample 2

        len = RECORD_BUFFER_SAMPLE_SIZE * 6 + RECORD_HEADER_SIZE;
        memset(ptr2, 0x0, len);

        acc_record.sample_count = RECORD_BUFFER_SAMPLE_SIZE;

        get_header(ptr2);
        ptr2 += RECORD_HEADER_SIZE;

        memcpy(ptr2, ptr + acc_record.sample_index * 6, (RECORD_BUFFER_SAMPLE_SIZE - acc_record.sample_index) * 6);
        memcpy(ptr2 + (RECORD_BUFFER_SAMPLE_SIZE - acc_record.sample_index) * 6, ptr, acc_record.sample_index * 6);

        ptr2 -= RECORD_HEADER_SIZE;

        end_of_buffer           = 1;
        save                    = 1;
        acc_record.state        = ACC_RECORD_CONTINUE;
        acc_record.buffer_stage = 1;

        break;

    case ACC_RECORD_CONTINUE:

        acc_record.sample_count++;

        if (end_of_buffer) {
            len = RECORD_BUFFER_SAMPLE_SIZE * 6;
            memcpy(ptr2, ptr, len);
            save = 1;
        }
        break;
    }

    if (end_of_buffer)
        acc_record.sample_index = 0;

    if (save) {
        acc_record.buffer_stage = !acc_record.buffer_stage;

        // copy ptr to the flash.
        // TO DO IN FUTURE: copy ptr2 in parallel
        flash_address = FLASH_RECORDS_START_ADDRESS + acc_record.record_num * RECORD_SIZE + acc_record.flash_address;

        acc_record.file_crc = CRC16_Calc(ptr2, len, acc_record.file_crc);

        memcpy(flash_buffer + 4, ptr2, len);
        flash_write_buffer(flash_buffer, flash_address, len);

        acc_record.flash_address += len;

        if ((acc_record.flash_address + RECORD_BUFFER_SAMPLE_SIZE * 6) >= (RECORD_SIZE - RECORD_TERMINATOR_SIZE)) {

            buzzer_train(3);
            terminal_buffer_lock();
            sprintf(alert_str, "\r\nAccident Recording: %d\r\n", acc_record.record_num);
            DisplayMessageWithTime(alert_str, strlen(alert_str), false);
            terminal_buffer_release();

            // CreateDebugEvent(EVENT_DEBUG_ACC_RECORD_COMPLETE, acc_record.record_id, 3, 0);

            // SendRecordBleAlert(acc_record.record_id);
            SendRecordAlert(acc_record.record_id);

            record_write_status(acc_record.record_num, RECORD_CLOSE_IND, 0);

            flash_address = FLASH_RECORDS_START_ADDRESS + (acc_record.record_num + 1) * RECORD_SIZE - RECORD_TERMINATOR_SIZE;
            flash_address += 2;

            // Program_Flash((unsigned char *)(&acc_record.file_crc), 2, flash_address);
            memcpy(flash_buffer + 4, (unsigned char *)(&acc_record.file_crc), 2);
            flash_write_buffer(flash_buffer, flash_address, 2);

            flash_address += 2;
            // Program_Flash((unsigned char *)(&acc_record.sample_count), 2, flash_address);
            memcpy(flash_buffer + 4, (unsigned char *)(&acc_record.sample_count), 2);
            flash_write_buffer(flash_buffer, flash_address, 2);

            record_create_new();
            // SetCpuClockSpeed(FAST_CLOCK_REQ_SRC_FLASH, 0);
        } else {
            // do nothing
        }
    }
}

void record_print(unsigned char record_num)
{
    xSemaphoreTake(tx_uart_semaphore, portMAX_DELAY);

    uint8_t *          buffer;
    uint32_t           flash_address;
    AccConvertedSample sample;
    uint16_t           i     = RECORD_HEADER_SIZE;
    uint16_t           count = 0;
    uint32_t           value;
    uint16_t           j;

    // DelaySleep(120, 0);

    /////////////////
    // read header //
    /////////////////

    flash_address = FLASH_RECORDS_START_ADDRESS + record_num * RECORD_SIZE;

    memset(flash_buffer, 0x00, 260);
    flash_read_buffer(flash_buffer, flash_address, 32);
    buffer = flash_buffer + 4;

    memcpy(&value, buffer, 4);

    // print header //

    terminal_buffer_lock();
    sprintf(alert_str, "Record %d: %d-%d-%d %d:%d:%d\r\n", value, buffer[7], buffer[8], buffer[9], buffer[4], buffer[5], buffer[6]);
    DisplayMessage(alert_str, strlen(alert_str), false);
    terminal_buffer_release();

    // print binary //

    i      = 0;
    buffer = flash_buffer + 4;

    while (i < RECORD_SIZE) {
        memset(flash_buffer, 0x00, 260);
        flash_read_buffer(flash_buffer, (flash_address + i), 256);

        j = 0;

        while (j < 256) {

            terminal_buffer_lock();

            value = buffer[j];
            sprintf(alert_str, "%02X ", value);
            DisplayMessage(alert_str, 0, false);
            nrf_delay_ms(1);
            j++;

            if ((j % 16) == 0) {
                DisplayMessage("\r\n", 0, false);
                nrf_delay_ms(1);
            }

            terminal_buffer_release();
        }

        i += 256;
    }

    // print samples //
    i = RECORD_HEADER_SIZE;

    while ((i + 6) < (RECORD_SIZE - RECORD_TERMINATOR_SIZE)) {
        memset(flash_buffer, 0x00, 10);
        flash_read_buffer(flash_buffer, (flash_address + i), 6);

        memcpy(&sample, buffer, 6);

        terminal_buffer_lock();
        sprintf(alert_str, "%d %d %d", sample.drive_direction, sample.turn_direction, sample.earth_direction);
        DisplayMessage(alert_str, 0, false);
        DisplayMessage("\r\n", 0, false);
        nrf_delay_ms(1);
        terminal_buffer_release();

        i += 6;
    }

    xSemaphoreGive(tx_uart_semaphore);
}

static void SendRecordAlert(uint32_t record_id)
{
    terminal_buffer_lock();
    sprintf(alert_str + 2, "@?REC,%d\r\n", record_id);
    PostBleAlert(alert_str);
    terminal_buffer_release();
}

uint32_t GetRandomNumber(void)
{
    uint32_t r = 0;

    sd_rand_application_vector_get((uint8_t *)&r, 4);
    r = r & 0xFFFFFFF;

    return r;
}