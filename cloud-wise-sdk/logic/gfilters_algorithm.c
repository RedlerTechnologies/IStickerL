
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

GFilterConfig filter_configs[GFILTER_NUM];
GFilterState  filter_states[GFILTER_NUM];

NRF_QUEUE_DEF(GFilterState, m_event_queue, MAX_EVENTS_IN_QUEUE, NRF_QUEUE_MODE_NO_OVERFLOW);

static void Process_GFilter(GFilterConfig *filter_config, GFilterState *filter_state, AccConvertedSample *sample);

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
    filter_config->positive      = 1;
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
    filter_config->positive      = 0;
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

    if (driver_behaviour_state.print_signal_mode)
        return;

    if (driver_behaviour_state.registration_mode)
        return;

    if (!driver_behaviour_state.calibrated)
        return;

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
            if (value > -filter_config->min_g)
                new_state_found = true;
        } else {
            if (value < filter_config->min_g)
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
                filter_state->state = GFILTER_AXIS_STATE_COMPLETED;
        }

        break;

    case GFILTER_AXIS_STATE_COMPLETED:
        // add inside queue

        /* ?????????
        if (device_config.buzzer_mode == BUZZER_MODE_DRIVER_BEHAVIOUR)
            buzzer_train(3);
            */

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

        if (filter_state->severity <= 1) {
            if (device_config.buzzer_mode == BUZZER_MODE_DEBUG)
                buzzer_train(1);
        } else {
            if (device_config.buzzer_mode >= BUZZER_MODE_ON)
                buzzer_train(filter_state->severity * 2);
        }

        // BLE alert //
        uint8_t code = filter_config->code;
        if ( !filter_config->positive && filter_config->axis == GFILTER_AXIS_Y)
          code = DRIVER_BEHAVIOR_SHARP_TURN_RIGHT;

        terminal_buffer_lock();
        sprintf(alert_str + 2, "@?DR,%d,%d\r\n", code, filter_state->severity);
        PostBleAlert(alert_str);
        terminal_buffer_release();

        nrf_queue_write(&m_event_queue, filter_state, 1);

        filter_state->state = GFILTER_AXIS_STATE_DELAY;
        // filter_state->duration_count = 0;
        // filter_state->energy         = 0;

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
    }
}