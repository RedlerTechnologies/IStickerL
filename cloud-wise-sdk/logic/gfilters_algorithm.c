
#include "gfilters_algorithm.h"

#include "FreeRTOS.h"
#include "ble/ble_services_manager.h"
#include "ble/ble_task.h"
#include "ble_file_transfer.h"
#include "commands.h"
#include "configuration.h"
#include "decoder.h"
#include "drivers/buzzer.h"
#include "drivers/lis3dh.h"
#include "event_groups.h"
#include "events.h"
#include "flash_data.h"
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
#include "nrf_queue.h"
#include "nrfx_gpiote.h"
#include "nrfx_log.h"
#include "recording.h"
#include "semphr.h"
#include "task.h"
#include "tracking_algorithm.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern DriverBehaviourState driver_behaviour_state;
extern DeviceConfiguration  device_config;
extern xSemaphoreHandle     clock_semaphore;
extern IStickerErrorBits    error_bits;

GFilterConfig filter_configs[GFILTER_NUM];
GFilterState  filter_states[GFILTER_NUM];

NRF_QUEUE_DEF(GFilterState, m_event_queue, MAX_EVENTS_IN_QUEUE, NRF_QUEUE_MODE_NO_OVERFLOW);

static void Process_GFilter(GFilterConfig *filter_config, GFilterState *filter_state, AccConvertedSample *sample);
static void Process_Bumper(void);
static void Process_Offroad(void);
static void send_ble_offroad_alert(uint8_t status);

void gfilter_init(void)
{
    GFilterConfig *filter_config;
    GFilterState * filter_state;
    uint8_t        i;

    filter_config                = &filter_configs[GFILTER_ACCELERATION];
    filter_config->axis          = GFILTER_AXIS_X;
    filter_config->min_duration  = ACCELERATION_MIN_DUR;
    filter_config->min_g         = ACCELERATION_MIN_G;
    filter_config->min_duration2 = ACCELERATION_MIN_MED_DUR;
    filter_config->min_g2        = ACCELERATION_MIN_MED_G;
    filter_config->min_duration3 = ACCELERATION_MIN_HIGH_DUR;
    filter_config->min_g3        = ACCELERATION_MIN_HIGH_G;
    filter_config->code          = DRIVER_BEHAVIOR_EVENT_ACCEL;
    filter_config->positive      = 1;
    strcpy(filter_config->name, "ACCEL");

    filter_config                = &filter_configs[GFILTER_BRAKES];
    filter_config->axis          = GFILTER_AXIS_X;
    filter_config->min_duration  = ACCELERATION_MIN_DUR;
    filter_config->min_g         = ACCELERATION_MIN_G;
    filter_config->min_duration2 = ACCELERATION_MIN_MED_DUR;
    filter_config->min_g2        = ACCELERATION_MIN_MED_G;
    filter_config->min_duration3 = ACCELERATION_MIN_HIGH_DUR;
    filter_config->min_g3        = ACCELERATION_MIN_HIGH_G;
    filter_config->code          = DRIVER_BEHAVIOR_EVENT_BRAKES;
    filter_config->positive      = 0;
    filter_config->positive      = 0;

    strcpy(filter_config->name, "BRAKES");

    filter_config                = &filter_configs[GFILTER_TURN_LEFT];
    filter_config->axis          = GFILTER_AXIS_Y;
    filter_config->min_duration  = TURN_MIN_DUR;
    filter_config->min_g         = TURN_MIN_G;
    filter_config->min_duration2 = TURN_MIN_MED_DUR;
    filter_config->min_g2        = TURN_MIN_MED_G;
    filter_config->min_duration3 = TURN_MIN_HIGH_DUR;
    filter_config->min_g3        = TURN_MIN_HIGH_G;
    filter_config->code          = DRIVER_BEHAVIOR_SHARP_TURN;
    filter_config->positive      = 0;
    strcpy(filter_config->name, "TLEFT");

    filter_config                = &filter_configs[GFILTER_TURN_RIGHT];
    filter_config->axis          = GFILTER_AXIS_Y;
    filter_config->min_duration  = TURN_MIN_DUR;
    filter_config->min_g         = TURN_MIN_G;
    filter_config->min_duration2 = TURN_MIN_MED_DUR;
    filter_config->min_g2        = TURN_MIN_MED_G;
    filter_config->min_duration3 = TURN_MIN_HIGH_DUR;
    filter_config->min_g3        = TURN_MIN_HIGH_G;
    filter_config->code          = DRIVER_BEHAVIOR_SHARP_TURN;
    filter_config->positive      = 1;
    strcpy(filter_config->name, "TRIGHT");

    for (i = 0; i < GFILTER_NUM; i++) {
        filter_state = &filter_states[i];
        memset(filter_state, 0x00, sizeof(GFilterState));
        filter_state->index = i;
    }
}

void Process_GFilters(AccConvertedSample *samples)
{
    static GFilterState event_state;
    GFilterConfig *     filter_config;
    GFilterState *      filter_state;
    uint32_t            duration;
    uint8_t             i, j;
    ret_code_t          status;

    if (driver_behaviour_state.registration_mode)
        return;

    if (!driver_behaviour_state.calibrated)
        return;

    Process_Offroad();

    Process_Bumper();
    duration = timeDiff(xTaskGetTickCount(), driver_behaviour_state.last_bumper_time);

    // ???????????????? disable bumper blocking for this version
    duration = 10000;

    if (duration >= 3000) { // block driver events on bumper
        filter_config = &filter_configs[GFILTER_ACCELERATION];
        filter_state  = &filter_states[GFILTER_ACCELERATION];

        for (i = 0; i < SAMPLE_BUFFER_SIZE; i++) {
            Process_GFilter(filter_config, filter_state, &samples[i]);
        }

        filter_config = &filter_configs[GFILTER_BRAKES];
        filter_state  = &filter_states[GFILTER_BRAKES];

        for (i = 0; i < SAMPLE_BUFFER_SIZE; i++) {
            Process_GFilter(filter_config, filter_state, &samples[i]);
        }

        filter_config = &filter_configs[GFILTER_TURN_LEFT];
        filter_state  = &filter_states[GFILTER_TURN_LEFT];

        for (i = 0; i < SAMPLE_BUFFER_SIZE; i++) {
            Process_GFilter(filter_config, filter_state, &samples[i]);
        }

        filter_config = &filter_configs[GFILTER_TURN_RIGHT];
        filter_state  = &filter_states[GFILTER_TURN_RIGHT];

        for (i = 0; i < SAMPLE_BUFFER_SIZE; i++) {
            Process_GFilter(filter_config, filter_state, &samples[i]);
        }
    }

    // write one event from queue into the flash

    status = nrf_queue_peek(&m_event_queue, &event_state);

    if (status == NRF_SUCCESS) {
        duration = timeDiff(xTaskGetTickCount(), event_state.timestamp) / 1000;

        if (duration > TIME_EVENT_IN_QUEUE) {
            status        = nrf_queue_read(&m_event_queue, &event_state, 1);
            filter_config = &filter_configs[event_state.index];
            CreateDriverBehaviourEvent(filter_config, &event_state);
        }
    }
}

static int16_t GetAxisValue(AccConvertedSample *sample, uint8_t code)
{
    int16_t value = 0;

    switch (code) {

    case GFILTER_AXIS_X:
        value = sample->drive_direction;
        break;

    case GFILTER_AXIS_Y:
        value = sample->turn_direction;
        break;

    case GFILTER_AXIS_Z:
        value = sample->earth_direction;
        break;
    }

    return value;
}

static void Process_Offroad(void)
{
    uint32_t bumper_history;
    uint32_t duration;
    uint8_t  count;
    uint8_t  percentage;
    uint8_t  i;
    bool     offroad = false;

    if (driver_behaviour_state.Gz >= 30)
        offroad = true;

    bumper_history = driver_behaviour_state.bumper_history;
    bumper_history = bumper_history << 1;

    if (offroad) {
        bumper_history |= 0x00000001;
    }

    driver_behaviour_state.bumper_history = bumper_history;

    count = 0;

    for (i = 0; i < 32; i++) {
        if (bumper_history & 0x00000001)
            count++;

        bumper_history = bumper_history >> 1;
    }

    percentage = count * 100 / 32;

    if (percentage > 25) {
        // offroaad
        driver_behaviour_state.offroad_stated_time = xTaskGetTickCount();

        if (!error_bits.Offroad) {
            // offroad started
            send_ble_offroad_alert(1);
        }

        error_bits.Offroad = 1;
    } else {
        duration = timeDiff(xTaskGetTickCount(), driver_behaviour_state.offroad_stated_time) / 1000;

        if (duration > 10) {

            if (error_bits.Offroad) {
                // offroad stops
                send_ble_offroad_alert(0);
            }

            error_bits.Offroad = 0;
        }
    }
}

static void send_ble_offroad_alert(uint8_t status)
{
    terminal_buffer_lock();
    sprintf(alert_str + 2, "@?OFFROAD,%d\r\n", status);
    PostBleAlert(alert_str);
    terminal_buffer_release();
}

static void Process_Bumper(void)
{
    static uint32_t measure_time = 0;
    static uint32_t bumper_count = 0;

    uint32_t duration;
    bool     bumper = false;

    if (device_config.filter_z > 0) {
        if (driver_behaviour_state.Gz > device_config.filter_z) {
            bumper = true;
        }
    }

    if (bumper) {
        driver_behaviour_state.last_bumper_time = xTaskGetTickCount();
        bumper_count++;

        terminal_buffer_lock();
        sprintf(alert_str + 2, "@?BUMPER,%d\r\n", driver_behaviour_state.Gz);
        PostBleAlert(alert_str);
        terminal_buffer_release();
    }

    duration = timeDiff(xTaskGetTickCount(), measure_time) / 1000;

    if (duration > 60) {
        if (bumper_count > 0)
            CreateGeneralEvent(bumper_count, EVENT_TYPE_BUMPER, 2);
        bumper_count = 0;
        measure_time = xTaskGetTickCount();
        ;
    }
}

static void Process_GFilter(GFilterConfig *filter_config, GFilterState *filter_state, AccConvertedSample *sample)
{
    int16_t  value;
    uint16_t value_pos;
    uint16_t n;
    bool     new_state_found = false;
    bool     negative_flag   = false;

    if (filter_config->min_duration == 0)
        return;

    value     = GetAxisValue(sample, filter_config->axis);
    value_pos = abs(value);

    negative_flag = filter_config->positive != 1;
    // if (filter_config->min_g < 0)
    //    negative_flag = true;

    switch (filter_state->state) {

    case GFILTER_AXIS_STATE_NONE:

        if (negative_flag) {
            if (value <= -filter_config->min_g)
                new_state_found = true;
        } else {
            if (value >= filter_config->min_g)
                new_state_found = true;
        }

        if (new_state_found) {
            filter_state->state          = GFILTER_AXIS_STATE_IDENTIFIED;
            filter_state->duration_count = 1;
            filter_state->energy         = value * value;
            filter_state->max_g          = value_pos;

            xSemaphoreTake(clock_semaphore, portMAX_DELAY);
            filter_state->event_time = GetTimeStampFromDate();
            xSemaphoreGive(clock_semaphore);
        }
        break;

    case GFILTER_AXIS_STATE_IDENTIFIED:

        if (value_pos > filter_state->max_g)
            filter_state->max_g = value_pos;

        if (negative_flag) {
            if (value > -filter_config->min_g / 2)
                new_state_found = true;
        } else {
            if (value < filter_config->min_g / 2)
                new_state_found = true;
        }

        filter_state->energy += value * value;

        if (new_state_found) {
            // check if duration is enough
            // ..

            n = filter_config->min_duration / SAMPLE_PERIOD;

            if (filter_state->duration_count > n)
                filter_state->state = GFILTER_AXIS_STATE_COMPLETED;
            else
                filter_state->state = GFILTER_AXIS_STATE_NONE;
        } else {
            filter_state->duration_count++;

            // duration_count is too long
            n = MAX_EVENT_DURATION / SAMPLE_PERIOD;

            if (filter_state->duration_count > n)
                filter_state->state = GFILTER_AXIS_STATE_COMPLETED2;
        }

        break;

    case GFILTER_AXIS_STATE_COMPLETED:
    case GFILTER_AXIS_STATE_COMPLETED2:
        // add inside queue

        // dont use the following line! activity (sleeping delay) is only done by
        // energy signal and not by driver behaviour events
        // driver_behaviour_state.last_activity_time = xTaskGetTickCount();

        terminal_buffer_lock();
        sprintf(alert_str, "\r\nEvent: %s\r\n", filter_config->name);
        DisplayMessage(alert_str, 0, false);
        terminal_buffer_release();

        filter_state->timestamp = xTaskGetTickCount();

        filter_state->energy         = sqrt(filter_state->energy);
        filter_state->duration_count = filter_state->duration_count * 1000 / ACC_SAMPLE_FREQ;

        filter_state->severity = 1;

        n = filter_config->min_duration2 / SAMPLE_PERIOD;
        if (filter_state->max_g >= filter_config->min_g2 && filter_state->duration_count >= n)
            filter_state->severity++;

        n = filter_config->min_duration3 / SAMPLE_PERIOD;
        if (filter_state->max_g >= filter_config->min_g3 && filter_state->duration_count >= n)
            filter_state->severity++;

        {
            uint8_t beeps = 0;

            switch (filter_state->severity) {
            case 2:
                if (device_config.buzzer_mode >= BUZZER_MODE_ON)
                    beeps = 2;
                break;

            case 3:
                if (device_config.buzzer_mode >= BUZZER_MODE_ON)
                    beeps = 6;
                break;

            default:
                if (device_config.buzzer_mode == BUZZER_MODE_DEBUG)
                    beeps = 1;
                break;
            }

            if (beeps > 0)
                buzzer_train(beeps);
        }

        // BLE alert //
        uint8_t code = filter_config->code;
        if (filter_config->positive && filter_config->axis == GFILTER_AXIS_Y)
            code = DRIVER_BEHAVIOR_SHARP_TURN_RIGHT;

        terminal_buffer_lock();
        sprintf(alert_str + 2, "@?DR,%d,%d\r\n", code, filter_state->severity);
        PostBleAlert(alert_str);
        terminal_buffer_release();

        nrf_queue_write(&m_event_queue, filter_state, 1);

        if (filter_state->state == GFILTER_AXIS_STATE_COMPLETED)
            filter_state->state = GFILTER_AXIS_STATE_DELAY;
        else
            filter_state->state = GFILTER_AXIS_STATE_DELAY2;

        break;

    case GFILTER_AXIS_STATE_DELAY:

        n = (DELAY_BETWEEN_GFILTER_EVENT_MSEC / SAMPLE_PERIOD);

        filter_state->duration_count++;

        if (filter_state->duration_count > n) {

            n = filter_state->index;
            memset(filter_state, 0x00, sizeof(GFilterState));
            filter_state->index = n;

            filter_state->state = GFILTER_AXIS_STATE_NONE;
        }

        break;

    case GFILTER_AXIS_STATE_DELAY2:

        if (value_pos < filter_config->min_g / 2) {
            filter_state->duration_count = 0;
            filter_state->state          = GFILTER_AXIS_STATE_DELAY;
        }

        break;
    }
}