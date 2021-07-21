#include "recording.h"

#include "FreeRTOS.h"
#include "ble/ble_services_manager.h"
#include "ble_file_transfer.h"
#include "configuration.h"
#include "decoder.h"
#include "drivers/buzzer.h"
#include "drivers/flash.h"
#include "drivers/lis3dh.h"
#include "event_groups.h"
#include "events.h"
#include "flash_data.h"
#include "hal/hal_boards.h"
#include "logic/serial_comm.h"
#include "monitor.h"
#include "nrf_delay.h"
#include "semphr.h"
#include "timers.h"
#include "tracking_algorithm.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static uint8_t flash_buffer[260];

extern xSemaphoreHandle     tx_uart_semaphore;
extern DriverBehaviourState driver_behaviour_state;
extern AccidentState        accident_state;
extern ScanResult           scan_result;
extern DeviceConfiguration  device_config;

xSemaphoreHandle acc_recording_semaphore;

static AccConvertedSample acc_samples[SAMPLE_BUFFER_SIZE];

EventGroupHandle_t event_save_recording;
EventGroupHandle_t event_acc_process_sample;
TimerHandle_t      sample_timer_handle;

#define NRF_LOG_MODULE_NAME recording
#define NRF_LOG_LEVEL CLOUD_WISE_DEFAULT_LOG_LEVEL
#include "nrfx_log.h"
NRF_LOG_MODULE_REGISTER();

EventGroupHandle_t event_sample_timer;
static uint8_t     sample_buffer[7];

#ifdef ACC_SAMPLE_FREQ_100HZ
// #define RECORD_SAMPLE_FREQ_CODE RECORD_SAMPLE_FREQ_100HZ
#endif

#ifdef ACC_SAMPLE_FREQ_200HZ
//#define RECORD_SAMPLE_FREQ_CODE RECORD_SAMPLE_FREQ_200HZ
#endif

#ifdef ACC_SAMPLE_FREQ_400HZ
//#define RECORD_SAMPLE_FREQ_CODE RECORD_SAMPLE_FREQ_400HZ
#endif

// TODO: chamge this define for each allowed samping rates
// bug in Back office. It read correctly only of the hz code is 50hz
// unremark the aboce section when the bug is fixed
#define RECORD_SAMPLE_FREQ_CODE RECORD_SAMPLE_FREQ_50HZ

static void record_create_new(void);
void SendRecordAlert(uint32_t record_id);

AccRecord acc_record;

// TODO: remove this callback as it not needed in DMA reading for accelerometer
void sample_timer_toggle_timer_callback(TimerHandle_t xTimer)
{
    BaseType_t xHigherPriorityTaskWoken, xResult;

    UNUSED_PARAMETER(xTimer);

    xHigherPriorityTaskWoken = pdFALSE;

    xResult = xEventGroupSetBitsFromISR(event_sample_timer, 0x01, &xHigherPriorityTaskWoken);

    if (xResult != pdFAIL) {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void calculate_sample(AccSample *acc_sample, uint8_t *buffer)
{
    signed short   x, y, z;
    signed short   x1, y1, z1;
    unsigned short mask = 0x8000;
    ret_code_t     err_code;

    x = 0;
    y = 0;
    z = 0;

    x |= (buffer[1] & 0xFF);
    x |= ((buffer[2] & 0xFF) << 8);

    y |= (buffer[3] & 0xFF);
    y |= ((buffer[4] & 0xFF) << 8);

    z |= (buffer[5] & 0xFF);
    z |= ((buffer[6] & 0xFF) << 8);

    x1 = ((x & 0xFFF0) >> 4);
    if (x & mask)
        x1 |= 0xF000;
    else
        x1 &= 0x0FFF;

    y1 = ((y & 0xFFF0) >> 4);
    if (y & mask)
        y1 |= 0xF000;
    else
        y1 &= 0x0FFF;

    z1 = ((z & 0xFFF0) >> 4);
    if (z & mask)
        z1 |= 0xF000;
    else
        z1 &= 0x0FFF;

    acc_sample->X = -x1 * 12;
    acc_sample->Y = -y1 * 12;
    acc_sample->Z = z1 * 12;
}

void ACC_CalibrateSample(AccSample *acc_sample_in, AccConvertedSample *acc_sample_out)
{
    DriverBehaviourState *state = &driver_behaviour_state;

    float x, y, z;
    float drive_direction, turn_direction, earth_direction;

    acc_sample_in->X -= state->calibrated_value.avg_value.X;
    acc_sample_in->Y -= state->calibrated_value.avg_value.Y;
    acc_sample_in->Z -= state->calibrated_value.avg_value.Z;

    /*
        x = -acc_sample_in->X;
        y = -acc_sample_in->Y;
        */

    x = acc_sample_in->X;
    y = acc_sample_in->Y;

    z = acc_sample_in->Z;

    x /= ACC_NORMALIZATION_VALUE;
    y /= ACC_NORMALIZATION_VALUE;
    z /= ACC_NORMALIZATION_VALUE;

    switch (state->calibrated_value.axis) {
    case 0:
        // error in calibration
        break;

    case 1:
        // error right now...
        break;

    case 2:
        drive_direction = y * cos(state->angle1);
        turn_direction  = x;
        earth_direction = z * sin(state->angle1);
        break;

    case 3:
        drive_direction = z * sin(state->angle1);
        turn_direction  = x;
        earth_direction = y * cos(state->angle1);
        break;
    }

    acc_sample_out->drive_direction = (signed short)(drive_direction * 100);
    acc_sample_out->earth_direction = (signed short)(earth_direction * 100);
    acc_sample_out->turn_direction  = (signed short)(turn_direction * 100);
}

void sampler_task(void *pvParameters)
{
    UNUSED_PARAMETER(pvParameters);

    vTaskDelay(3000);
    lis3dh_configure_fifo(); // ??????????

    while (1) {
        vTaskSuspend(NULL);

        lis3dh_int_handler();
    }
}

/*
void sampler_task(void *pvParameter)
{
    static AccSample          acc_sample;
    static AccConvertedSample acc_sample1;
    static uint16_t           block_length;

    AccConvertedSample *acc_sample_array;

    EventBits_t uxBits;
    uint16_t    index  = 0;
    uint8_t     count  = 0;
    uint8_t     status = 0;

    block_length = sizeof(AccConvertedSample) * SAMPLE_BUFFER_SIZE;

    sample_timer_handle = xTimerCreate("SAMPLES", TIMER_PERIOD, pdTRUE, NULL, sample_timer_toggle_timer_callback);
    UNUSED_VARIABLE(xTimerStart(sample_timer_handle, 0));

    vTaskDelay(3000);

    while (1) {

        monitor_task_set(TASK_MONITOR_BIT_TRACKING);

        uxBits = xEventGroupWaitBits(event_sample_timer, 0x01, pdTRUE, pdFALSE, 100);

        if (uxBits) {
            lis3dh_read_buffer(sample_buffer, 7, (0x27 | 0x80));

            status = sample_buffer[0];

            if (!acc_record.accident_saving) {
                if (status & 0xF0) {
                    acc_record.acc_overrrun_count++;
                }
            }

            if (status & 0x08) {
            } else {
                continue;
            }

            calculate_sample(&acc_sample, sample_buffer);

            if (driver_behaviour_state.calibrated && driver_behaviour_state.track_state != TRACKING_STATE_SLEEP) {
                ACC_CalibrateSample(&acc_sample, &acc_sample1);
            } else {
                memcpy(&acc_sample1, &acc_sample, sizeof(AccSample));
            }

            acc_samples[count] = acc_sample1;

            count++;

            if (count >= SAMPLE_BUFFER_SIZE) {
                count = 0;

                xSemaphoreTake(acc_recording_semaphore, portMAX_DELAY);

                memcpy(driver_behaviour_state.current_samples, acc_samples, block_length);
                xEventGroupSetBits(event_acc_process_sample, 0x01);

                if (!acc_record.accident_saving) {
                    index = acc_record.sample_index;

                    if (acc_record.accident_stage) {
                        acc_sample_array = &acc_record.samples_after_event[index][0];

                    } else {
                        acc_sample_array = &acc_record.samples_before_event[index][0];
                    }

                    memcpy(acc_sample_array, acc_samples, block_length);
                    index++;

                    if (acc_record.accident_stage) {

                        if (index >= NUM_SAMPLE_BLOCK_AFTER_EVENT) {
                            // complete recording
                            // ..

                            acc_record.accident_stage      = false;
                            acc_record.accident_identified = false;
                            acc_record.accident_saving     = true;

                            xEventGroupSetBits(event_save_recording, 0x01);

                            index = 0;
                        }

                    } else {
                        if (acc_record.sample_size_before < NUM_SAMPLE_BLOCK_BEFORE_EVENT)
                            acc_record.sample_size_before++;

                        acc_record.last_sample_index = index;

                        if (index >= NUM_SAMPLE_BLOCK_BEFORE_EVENT)
                            index = 0;

                        if (acc_record.accident_identified) {
                            acc_record.accident_stage = true;
                            index                     = 0;
                        }
                    }

                    acc_record.sample_index = index;
                }

                xSemaphoreGive(acc_recording_semaphore);
            }

        } else {
            // failure in interrupt
        }
    }
}
*/

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

    xSemaphoreTake(acc_recording_semaphore, portMAX_DELAY);

    // point to the oldest record date here
    acc_record.record_num = min_id;

    acc_record.accident_stage      = false;
    acc_record.accident_identified = false;
    acc_record.accident_saving     = false;

    xSemaphoreGive(acc_recording_semaphore);

    record_create_new();
}

static void record_create_new(void)
{
    uint32_t flash_address;
    uint8_t  i;
    uint8_t  ratio;

    xSemaphoreTake(acc_recording_semaphore, portMAX_DELAY);

    memset(acc_record.samples_before_event, 0x0, SAMPLE_BUFFER_SIZE * NUM_SAMPLE_BLOCK_BEFORE_EVENT * sizeof(AccConvertedSample));
    memset(acc_record.samples_after_event, 0x0, SAMPLE_BUFFER_SIZE * NUM_SAMPLE_BLOCK_AFTER_EVENT * sizeof(AccConvertedSample));

    acc_record.flash_address = 0;
    acc_record.record_id     = 0;
    acc_record.file_crc      = 0;

    acc_record.sample_size_before = 0;
    acc_record.accident_saving    = false;

    acc_record.record_num++;
    if (acc_record.record_num >= MAX_RECORDS)
        acc_record.record_num = 0;

    xSemaphoreGive(acc_recording_semaphore);

    // delete region (sector) here

    flash_address = FLASH_RECORDS_START_ADDRESS + acc_record.record_num * RECORD_SIZE;
    ratio         = (RECORD_SIZE / FLASH_SECTOR_SIZE);

    for (i = 0; i < ratio; i++) {
        flash_erase_sector(flash_address);
        flash_address += FLASH_SECTOR_SIZE;
    }
}

void print_record(uint32_t rec, uint16_t index, uint8_t pending_status)
{
    terminal_buffer_lock();
    sprintf(alert_str, "\r\nrec=%d, idx=%d, pend=%d\r\n", rec, index, pending_status);
    DisplayMessage(alert_str, 0, false);
    terminal_buffer_release();
}

uint8_t record_scan_for_new_records(bool forced)
{
    static uint8_t read_buffer[12];
    uint8_t *      buffer;
    uint32_t       flash_address;
    uint32_t       record_id;
    int16_t        index = -9;
    uint32_t       duration;
    uint16_t       max_record_id = 0;
    uint8_t        i;
    uint8_t        pending_counter = 0;
    uint8_t        pending_status;
    bool           flag;

    index = -1;

    acc_record.last_found_record_time = xTaskGetTickCount();

    for (i = 0; i < MAX_RECORDS; i++) {
        flash_address = FLASH_RECORDS_START_ADDRESS + (i + 1) * RECORD_SIZE - RECORD_TERMINATOR_SIZE;

        flag = flash_read_buffer(read_buffer, flash_address, 8);
        if (flag) {
            buffer = read_buffer + 4;
            // if record is completed but not sent

            pending_status = (buffer[0] == 0 && buffer[1] == 0xFF);
            // if (buffer[0] == 0 && buffer[1] == 0xFF) {
            if (pending_status | forced) {
                if (pending_status)
                    pending_counter++;

                flash_address = FLASH_RECORDS_START_ADDRESS + i * RECORD_SIZE;

                flag = flash_read_buffer(read_buffer, flash_address, 4);

                if (flag) {
                    memcpy((uint8_t *)&record_id, read_buffer + 4, 4);

                    if (forced) {
                        print_record(record_id, i, pending_status);

                        /*
                            terminal_buffer_lock();
                            sprintf(alert_str, "\r\npending=%d, idx=%d\r\n", record_id, i);
                            DisplayMessage(alert_str, 0, false);
                            terminal_buffer_release();*/
                    }

                    if (record_id > max_record_id) {
                        index         = i;
                        max_record_id = record_id;
                    }
                }
            }
        }
    }

    if (index >= 0 && !forced) {

        if (((ble_services_is_connected()) && (driver_behaviour_state.accident_state != ACCIDENT_STATE_IDENTIFIED) &&
             (!in_sending_file())) ||
            forced) {
            SendRecordAlert(record_id);
        }
    }

    print_record(pending_counter, 0, -1);

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
    xSemaphoreTake(acc_recording_semaphore, portMAX_DELAY);

    acc_record.record_reason = reason;

    acc_record.record_id = scan_result.write_marker.event_id;
    // acc_record.record_id            = GetRandomNumber();

    acc_record.acc_overrrun_count  = 0;
    acc_record.accident_identified = true;

    xSemaphoreGive(acc_recording_semaphore);
}

void DeleteRecord(uint8_t record_num)
{
    uint32_t flash_address;

    flash_address = FLASH_RECORDS_START_ADDRESS + (record_num)*RECORD_SIZE;
    flash_erase_sectors_in_range(flash_address, (flash_address + RECORD_SIZE));

    terminal_buffer_lock();
    sprintf(alert_str, "\r\nRecord deleted: idx=%d\r\n", record_num);
    DisplayMessageWithTime(alert_str, strlen(alert_str), false);
    terminal_buffer_release();
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

    header[13] = RECORD_SAMPLE_FREQ_CODE;
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

void close_recording(void)
{
    uint32_t flash_address;
    uint16_t len;
    int16_t  i;
    int16_t  j;

    if (xEventGroupWaitBits(event_save_recording, 0x01, pdTRUE, pdFALSE, 1) == 0)
        return;

    len = SAMPLE_BUFFER_SIZE * sizeof(AccConvertedSample);

    flash_address = FLASH_RECORDS_START_ADDRESS + (acc_record.record_num) * RECORD_SIZE;

    get_header(flash_buffer + 4);
    flash_write_buffer(flash_buffer, flash_address, RECORD_HEADER_SIZE);
    flash_address += RECORD_HEADER_SIZE;

    for (i = acc_record.sample_size_before - 1; i >= 0; i--) {
        j = (acc_record.last_sample_index - i - 1);

        if (j < 0)
            j += NUM_SAMPLE_BLOCK_BEFORE_EVENT;

        memcpy(flash_buffer + 4, &acc_record.samples_before_event[j][0], len);
        flash_write_buffer(flash_buffer, flash_address, len);

        flash_address += len;
    }

    for (i = 0; i < NUM_SAMPLE_BLOCK_AFTER_EVENT; i++) {

        memcpy(flash_buffer + 4, &acc_record.samples_after_event[i][0], len);
        flash_write_buffer(flash_buffer, flash_address, len);

        flash_address += len;
    }

    // CreateDebugEvent(EVENT_DEBUG_ACC_RECORD_COMPLETE, acc_record.record_id, 3, 0);

    SendRecordAlert(acc_record.record_id);

    record_write_status(acc_record.record_num, RECORD_CLOSE_IND, 0);

    flash_address = FLASH_RECORDS_START_ADDRESS + (acc_record.record_num + 1) * RECORD_SIZE - RECORD_TERMINATOR_SIZE;
    flash_address += 2;

    memcpy(flash_buffer + 4, (unsigned char *)(&acc_record.file_crc), 2);
    flash_write_buffer(flash_buffer, flash_address, 2);

    flash_address += 2;
    memcpy(flash_buffer + 4, (unsigned char *)(&acc_record.sample_count), 2);
    flash_write_buffer(flash_buffer, flash_address, 2);

    if (device_config.buzzer_mode == BUZZER_MODE_DEBUG)
        buzzer_train(3);
    terminal_buffer_lock();
    sprintf(alert_str, "\r\nRecord Saved: idx=%d\r\n", acc_record.record_num);
    DisplayMessageWithTime(alert_str, strlen(alert_str), false);
    terminal_buffer_release();
    vTaskDelay(10);

    terminal_buffer_lock();

    if (acc_record.acc_overrrun_count) {
        sprintf(alert_str, "\r\nmissed samples=%d\r\n", acc_record.acc_overrrun_count);
        DisplayMessage(alert_str, strlen(alert_str), false);
    } else {
        DisplayMessage("\r\nNo missed samples\r\n", 0, false);
    }

    terminal_buffer_release();

    CreateAccidentEvent();
    record_create_new();
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
        monitor_task_set_all();

        j = 0;

        terminal_buffer_lock();

        while (j < 256) {

            value = buffer[j];
            sprintf(alert_str, "%02X ", value);
            DisplayMessage(alert_str, 0, false);
            nrf_delay_ms(1);
            j++;

            if ((j % 16) == 0) {
                DisplayMessage("\r\n", 0, false);
                terminal_buffer_release();
                vTaskDelay(20);
                terminal_buffer_lock();
            }
        }

        terminal_buffer_release();

        i += 256;
    }

    // print samples //
    i = RECORD_HEADER_SIZE;

    while ((i + 6) < (RECORD_SIZE - RECORD_TERMINATOR_SIZE)) {
        memset(flash_buffer, 0x00, 10);
        flash_read_buffer(flash_buffer, (flash_address + i), 6);
        monitor_task_set_all();

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

void SendRecordAlert(uint32_t record_id)
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

unsigned char check_stuck_record(unsigned char record_num)
{
    if (acc_record.last_sent_record_num != record_num) {
        acc_record.last_sent_record_num       = record_num;
        acc_record.last_sent_record_num_count = 0;
    } else {
        if (acc_record.last_sent_record_num_count > 3) {
            DeleteRecord(record_num);
            acc_record.last_sent_record_num_count = 0;
            return 0;
        }
    }

    return 1;
}