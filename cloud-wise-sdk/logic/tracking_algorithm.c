#include "tracking_algorithm.h"

#include "FreeRTOS.h"
#include "ble/ble_services_manager.h"
#include "ble/ble_task.h"
#include "commands.h"
#include "configuration.h"
#include "drivers/buzzer.h"
#include "drivers/flash.h"
#include "drivers/lis3dh.h"
#include "event_groups.h"
#include "events.h"
#include "float.h"
#include "hal/hal.h"
#include "hal/hal_boards.h"
#include "hal/hal_drivers.h"
#include "logic/clock.h"
#include "logic/serial_comm.h"
#include "monitor.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_power.h"
#include "nrf_pwr_mgmt.h"
#include "nrfx_gpiote.h"
#include "nrfx_log.h"
#include "recording.h"
#include "task.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

EventGroupHandle_t     event_acc_sample;
static TimerHandle_t   sample_timer_handle;
static CalibratedValue calibrated_value __attribute__((section(".non_init")));

AccSample acc_samples[SAMPLE_BUFFER_SIZE];

extern uint8_t             Acc_Table[];
extern uint8_t             Acc_Sleep_Table[];
extern DeviceConfiguration device_config;
extern uint32_t            reset_count_x;

static uint8_t sample_buffer[7];

DriverBehaviourState driver_behaviour_state;

void           calculate_sample(AccSample *acc_sample, uint8_t *buffer);
unsigned short GetGValue(AccConvertedSample *sample);
void           CalibrateAllSamples(void);
void           ProcessDrivingState(void);
void           ACC_CalibrateSample(AccSample *acc_sample_in, AccConvertedSample *acc_sample_out);
void           Process_Accident(DriverBehaviourState *state, AccConvertedSample *sample);
void           Process_Calibrate(void);
void           InitWakeupAlgorithm(void);
signed short   calculate_accident_hit_angle(AccConvertedSample *sample);
bool           ProcessWakeupState(void);
bool           CheckNoActivity(void);

void clear_calibration(void) { memset(&calibrated_value, 0x00, sizeof(CalibratedValue)); }

void copy_calibration(void) { memcpy(&driver_behaviour_state.calibrated_value, &calibrated_value, sizeof(CalibratedValue)); }

void sample_timer_toggle_timer_callback(void *pvParameter)
{
    BaseType_t xHigherPriorityTaskWoken, xResult;

    UNUSED_PARAMETER(pvParameter);

    xHigherPriorityTaskWoken = pdFALSE;

    xResult = xEventGroupSetBitsFromISR(event_acc_sample, 0x01, &xHigherPriorityTaskWoken);

    if (xResult != pdFAIL) {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

unsigned char wait_for_sample(void)
{
    EventBits_t uxBits;

    uxBits = xEventGroupWaitBits(event_acc_sample, 0x01, pdTRUE, pdFALSE, 100);

    if (event_acc_sample)
        return 1;
    else
        return 0;
}

void driver_behaviour_task(void *pvParameter)
{
    static unsigned  duration;
    static AccSample acc_sample;
    uint8_t          count = 0;
    bool             need_sleep;

    buzzer_train(1);

    ble_services_init_0();

    LoadConfiguration();
    ble_services_init();

#ifdef BLE_ADVERTISING
    ble_services_advertising_start();
#endif

    sample_timer_handle = xTimerCreate("SAMPLES", TIMER_PERIOD, pdTRUE, NULL, (TimerCallbackFunction_t)sample_timer_toggle_timer_callback);
    UNUSED_VARIABLE(xTimerStart(sample_timer_handle, 0));

    configure_acc(Acc_Table, ACC_TABLE_DRIVER_SIZE);
    // configure_acc(Acc_Sleep_Table, ACC_TABLE_SLEEP_SIZE);

    // init glabal state variables //

    driver_behaviour_state.time_synced           = false;
    driver_behaviour_state.record_triggered      = false;
    driver_behaviour_state.track_state           = TRACKING_STATE_WAKEUP;
    driver_behaviour_state.stop_advertising_time = 0;
    driver_behaviour_state.store_calibration     = false;

    InitWakeupAlgorithm();

    driver_behaviour_state.sleep_delay_time = 40;

    terminal_buffer_lock();
    sprintf(alert_str, "\r\n\r\nISticker-L version: %d.%d.%d - reset=%d\r\n\r\n", APP_MAJOR_VERSION, APP_MINOR_VERSION, APP_BUILD,
            reset_count_x);
    DisplayMessage(alert_str, 0);
    terminal_buffer_release();

    // test external flash
    uint16_t flash_id = flash_read_manufacture_id();
    NRFX_LOG_INFO("%s Flash ID 0x%04X", __func__, flash_id);

    record_init();

    while (1) {

        monitor_task_set(TASK_MONITOR_BIT_TRACKING);

        if (wait_for_sample()) {
            lis3dh_read_buffer(sample_buffer, 7, (0x27 | 0x80));
            calculate_sample(&acc_sample, sample_buffer);

            // add the sample to a sample array
            acc_samples[count] = acc_sample;
            count++;

            if (count >= SAMPLE_BUFFER_SIZE)
                count = 0;
            else
                continue;

        } else {
            // failure in interrupt
        }

        ///////////////////////////
        // process block of data //
        ///////////////////////////

        // handle last 32 samples

        need_sleep = false;

        if (!driver_behaviour_state.calibrated) {
            Process_Calibrate();
        } else {
            CalibrateAllSamples();

            switch (driver_behaviour_state.track_state) {
            case TRACKING_STATE_WAKEUP:
                if (ProcessWakeupState()) {
                    driver_behaviour_state.track_state        = TRACKING_STATE_ROUTE;
                    driver_behaviour_state.last_activity_time = xTaskGetTickCount();
                    driver_behaviour_state.sleep_delay_time   = 600; // ??????????? 120;

                    buzzer_train(2);

                    DisplayMessage("\r\nStart route\r\n", 0);
                    CreateGeneralEvent(0, EVENT_TYPE_START_ROUTE, 1);
                    continue;
                } else {
                    need_sleep = CheckNoActivity();

                    if (need_sleep) {
                        CreateGeneralEvent(LOG_FALSE_WAKEUP, EVENT_TYPE_LOG, 2);
                    }
                }

                break;

            case TRACKING_STATE_ROUTE:
                ProcessDrivingState();

                need_sleep = CheckNoActivity();

                if (need_sleep) {
                    CreateEndRouteEvent();
                    CreateGeneralEvent(LOG_SLEEP_BY_NO_MOVEMENT, EVENT_TYPE_LOG, 2);
                }
                break;

            case TRACKING_STATE_SLEEP:

                break;
            }
        }

        if (need_sleep) {
            DisplayMessage("\r\nSleeping...\r\n", 0);

            ble_services_disconnect();

            // need after creating end of route immediate events
            vTaskDelay(10000);

            DisplayMessageWithTime("Sleep\r\n", 0);
            buzzer_long(1200);
            vTaskDelay(2000);

            isticker_bsp_board_sleep();

            nrf_gpio_pin_sense_t sense = NRF_GPIO_PIN_SENSE_LOW;
            nrf_gpio_cfg_sense_set(HAL_LIS3DH_INT2, sense);

            SleepCPU(true);
        }
    }
}

void SleepCPU(bool with_memory_retention)
{

    uint8_t i;

#ifdef BLE_ADVERTISING

    for (i = 0; i < 8; i++) // Retain RAM 0 - RAM 7
    {
        sd_power_ram_power_set(i, (POWER_RAM_POWER_S0POWER_On << POWER_RAM_POWER_S0POWER_Pos) |
                                      (POWER_RAM_POWER_S1POWER_On << POWER_RAM_POWER_S1POWER_Pos) |
                                      (POWER_RAM_POWER_S0RETENTION_On << POWER_RAM_POWER_S0RETENTION_Pos) |
                                      (POWER_RAM_POWER_S1RETENTION_On << POWER_RAM_POWER_S1RETENTION_Pos));
        // if (ret_value != NRF_SUCCESS) {
        //    ret_value = 1;
        //}
    }

#endif

    nrf_power_system_off();

    while (1) {
        nrf_pwr_mgmt_run();
    }
}

bool CheckNoActivity(void)
{
    uint32_t duration;
    int16_t  time_to_sleep;
    bool     need_sleep = false;

#ifdef SLEEP_DISABLE
    return false;
#endif

    uint16_t timeout_in_sec = driver_behaviour_state.sleep_delay_time;

    duration = timeDiff(xTaskGetTickCount(), driver_behaviour_state.last_activity_time) / 1000;

    time_to_sleep                                    = timeout_in_sec - duration;
    driver_behaviour_state.time_to_sleep_left_in_sec = time_to_sleep;

    if (time_to_sleep <= 0) {

        need_sleep = true;
    }

    return need_sleep;
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

void CalibrateAllSamples(void)
{
    AccSample *        sample;
    AccConvertedSample sample_out;

    for (unsigned char i = 0; i < SAMPLE_BUFFER_SIZE; i++) {
        sample = &acc_samples[i];
        ACC_CalibrateSample(sample, &sample_out);

        memcpy((unsigned char *)sample, (unsigned char *)(&sample_out), 6);
    }
}

void ProcessDrivingState(void)
{
    static uint8_t ble_buffer[16];

    DriverBehaviourState *state = &driver_behaviour_state;
    AccConvertedSample *  sample;

    for (unsigned char i = 0; i < SAMPLE_BUFFER_SIZE; i++) {
        sample = (AccConvertedSample *)&acc_samples[i];

        if (i == 0) {
            memset(ble_buffer, 0x00, 16);
            memcpy(ble_buffer, sample, sizeof(AccConvertedSample));
            ble_services_notify_acc(ble_buffer, 16);
        }

        Process_Accident(state, sample);
    }
}

void InitWakeupAlgorithm(void)
{
    driver_behaviour_state.movement_count      = 0;
    driver_behaviour_state.movement_test_count = 0;
}

bool ProcessWakeupState(void)
{
    DriverBehaviourState *state = &driver_behaviour_state;
    AccConvertedSample *  sample;
    bool                  sense;
    bool                  sense1 = false;
    bool                  found  = false;

#ifdef SLEEP_DISABLE
    return true;
#endif

    for (unsigned char i = 0; i < SAMPLE_BUFFER_SIZE; i++) {
        sample = (AccConvertedSample *)&acc_samples[i];
        sense  = false;

        if (sample->turn_direction >= ACC_MIN_DRIVE_VALUE)
            sense = true;

        if (sample->drive_direction >= ACC_MIN_DRIVE_VALUE)
            sense = true;

        if (sense) {
            sense1 = true;

            if (state->movement_count == 0)
                InitWakeupAlgorithm();
            state->movement_count++;
        }
    }

    state->movement_test_count++;

    if (state->movement_test_count >= 15) // about 5 seconds
    {
        terminal_buffer_lock();
        sprintf(alert_str, "\r\n\r\nmovements# %d\r\n\r\n", state->movement_count);
        DisplayMessage(alert_str, 0);
        terminal_buffer_release();

        if (state->movement_count >= 5) {
            found = true;
        }
        InitWakeupAlgorithm();
    } else if (sense1) {
        terminal_buffer_lock();
        sprintf(alert_str, "\r\n\r\nm# %d\r\n\r\n", state->movement_count);
        DisplayMessage(alert_str, 0);
        terminal_buffer_release();
    }

    return found;
}

void ACC_CalibrateSample(AccSample *acc_sample_in, AccConvertedSample *acc_sample_out)
{
    DriverBehaviourState *state = &driver_behaviour_state;

    float x, y, z;
    float drive_direction, turn_direction, earth_direction;

    acc_sample_in->X -= state->calibrated_value.avg_value.X;
    acc_sample_in->Y -= state->calibrated_value.avg_value.Y;
    acc_sample_in->Z -= state->calibrated_value.avg_value.Z;

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

void Process_Calibrate(void)
{
    DriverBehaviourState *state = &driver_behaviour_state;

    float         x, y, z;
    float         angle1, angle2, temp;
    AccSample *   sample;
    unsigned char i;

    for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++) {
        sample = &acc_samples[i];

        state->sum_x += sample->X;
        state->sum_y += sample->Y;
        state->sum_z += sample->Z;
    }

    state->block_count++;

    if (state->block_count >= (3 * 5)) {
        state->block_count *= SAMPLE_BUFFER_SIZE;

        state->sum_x /= state->block_count;
        state->sum_y /= state->block_count;
        state->sum_z /= state->block_count;

        state->calibrated_value.avg_value.X = state->sum_x;
        state->calibrated_value.avg_value.Y = state->sum_y;
        state->calibrated_value.avg_value.Z = state->sum_z;

        x = state->calibrated_value.avg_value.X;
        y = state->calibrated_value.avg_value.Y;
        z = state->calibrated_value.avg_value.Z;

        x /= ACC_NORMALIZATION_VALUE;
        y /= ACC_NORMALIZATION_VALUE;
        z /= ACC_NORMALIZATION_VALUE;

        angle2 = asin(x);
        temp   = angle2 * 180 / PI;

        angle1 = atan(y / z);

        if (angle1 < PI * 45 / 180) {
            // y is the driving axis
            state->calibrated_value.axis = 2;
        } else {
            // z is the driving axis
            state->calibrated_value.axis = 3;
        }

        if (driver_behaviour_state.store_calibration) {
            memcpy(&calibrated_value, &state->calibrated_value, sizeof(CalibratedValue));
            state->tampered = false;
        }

        driver_behaviour_state.store_calibration = false;
        state->block_count                       = 0;
        state->calibrated                        = true;
        buzzer_train(5);

        // sending calibrate alert
        terminal_buffer_lock();
        sprintf(alert_str + 2, "@?C,%d,%d,%d\r\n", 1, -2, 0); // ???????????
        PostBleAlert(alert_str);
        terminal_buffer_release();
    }
}

void Process_Accident(DriverBehaviourState *state, AccConvertedSample *sample)
{
    unsigned short value;

    switch (state->accident_state) {
    case ACCIDENT_STATE_NONE:

        if (sample->turn_direction >= ACC_MIN_ACCIDENT_VALUE)
            state->accident_state = ACCIDENT_STATE_STARTED;

        if (sample->drive_direction >= ACC_MIN_ACCIDENT_VALUE)
            state->accident_state = ACCIDENT_STATE_STARTED;

        if (driver_behaviour_state.record_triggered)
            state->accident_state = ACCIDENT_STATE_STARTED;

        if (state->accident_state == ACCIDENT_STATE_STARTED) {
            state->accident_sample_count = 1;
            state->max_g                 = GetGValue(sample);

            state->last_activity_time = xTaskGetTickCount();
        }

        break;

    case ACCIDENT_STATE_STARTED:

        state->accident_sample_count++;
        value = GetGValue(sample);

        if (value > state->max_g) {
            state->max_g                     = value;
            state->sample_in_drive_direction = sample->drive_direction;
            state->sample_in_turn_direction  = sample->turn_direction;
            state->hit_angle                 = calculate_accident_hit_angle(sample);
        }

        if (state->accident_sample_count >= MIN_SAMPLES_FOR_ACCIDENT) {
            if (state->max_g >= MIN_G_FOR_ACCIDENT_EVENT || driver_behaviour_state.record_triggered) {
                /////////////////////////
                // accident identified //
                /////////////////////////

                state->accident_state                   = ACCIDENT_STATE_IDENTIFIED;
                driver_behaviour_state.record_triggered = false;

                record_trigger(0);
            } else {
                state->accident_state = ACCIDENT_STATE_NONE;
            }
        }

        break;

    case ACCIDENT_STATE_IDENTIFIED:

        if (state->accident_sample_count == MIN_SAMPLES_FOR_ACCIDENT) {
            // sending calibrate alert
            terminal_buffer_lock();
            sprintf(alert_str + 2, "@?X,%d,%d\r\n", state->max_g, state->hit_angle);
            PostBleAlert(alert_str);
            terminal_buffer_release();

            CreateAccidentEvent();

            // beep a buzzer
            buzzer_long(4000);
        }

        state->accident_sample_count++;
        // continue recording
        // ..

        // ~30 seconds delay between accident
        if (state->accident_sample_count >= 2000)
            state->accident_state = ACCIDENT_STATE_NONE;

        break;
    }

    record_add_sample(sample);
}

signed short calculate_accident_hit_angle(AccConvertedSample *sample)
{
    float value;

    if (sample->drive_direction == 0)
        sample->drive_direction = 1;

    value = atan((float)sample->turn_direction / sample->drive_direction);
    value = value * 180 / PI;

    if (sample->drive_direction < 0) {
        if (sample->turn_direction > 0)
            value = 360 + value;
    } else {
        value = 180 + value;
    }

    return (short)value;
}

unsigned short GetGValue(AccConvertedSample *sample)
{
    unsigned int value;

    value = sample->drive_direction * sample->drive_direction;
    value += sample->turn_direction * sample->turn_direction;

    value = sqrt(value);
    return value;
}