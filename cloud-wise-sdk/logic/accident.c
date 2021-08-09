#include "accident.h"

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
#include "gfilters_algorithm.h"
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
#include "task.h"
#include "tracking_algorithm.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern DriverBehaviourState driver_behaviour_state;
extern DeviceConfiguration device_config;

signed short calculate_accident_hit_angle(AccConvertedSample *sample) {
  float value;
  signed short v1;

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

  v1 = (short)value;

  /*
        // patch (reverse direction)
        v1 -= 180;
        if (v1 < 0)
            v1 += 360;
     */

  return v1;
}

unsigned short GetGValue(AccConvertedSample *sample) {
  unsigned int value;

  value = sample->drive_direction * sample->drive_direction;
  value += sample->turn_direction * sample->turn_direction;
  // value += sample->earth_direction * sample->earth_direction;

  value = sqrt(value);

  return value;
}

void Process_Accident(DriverBehaviourState *state, AccConvertedSample *sample) {

  static uint32_t last_print_time = 0;
  uint32_t duration;
  uint16_t value;

  if (driver_behaviour_state.print_signal_mode)
    return;

  if (driver_behaviour_state.registration_mode)
    return;

  if (!driver_behaviour_state.calibrated)
    return;

  if (device_config.AccidentG == 0)
    return;

  switch (state->accident_state) {
  case ACCIDENT_STATE_NONE:

    if (is_bumper_occured())
      break;

    if (abs(sample->turn_direction) >= ACC_MIN_ACCIDENT_VALUE)
      state->accident_state = ACCIDENT_STATE_STARTED;

    if (abs(sample->drive_direction) >= ACC_MIN_ACCIDENT_VALUE)
      state->accident_state = ACCIDENT_STATE_STARTED;

    /*
                if (abs(sample->earth_direction) >= ACC_MIN_ACCIDENT_VALUE)
                    state->accident_state = ACCIDENT_STATE_STARTED;
        */

    if (driver_behaviour_state.record_triggered)
      state->accident_state = ACCIDENT_STATE_STARTED;

    if (state->accident_state == ACCIDENT_STATE_STARTED) {
      state->accident_sample_count = 1;
      state->sample_on_gmax = *sample;
      state->max_g = GetGValue(sample);
      state->sum_g_accident = state->max_g;
      state->hit_angle = calculate_accident_hit_angle(sample);
    }

    break;

  case ACCIDENT_STATE_STARTED:

    state->accident_sample_count++;
    value = GetGValue(sample);
    state->sum_g_accident += value;

    if (value > state->max_g) {
      state->max_g = value;
      state->sample_on_gmax = *sample;
    }

    // bumper block accident
    if (is_bumper_occured())
      state->accident_state = ACCIDENT_STATE_NONE;

    if (state->accident_sample_count >= MIN_SAMPLES_FOR_ACCIDENT) {

      if (device_config.profile_code == PROFILE_LAB) {
        duration = timeDiff(xTaskGetTickCount(), last_print_time);

        if (duration > 2500) {
          state->hit_angle = calculate_accident_hit_angle(&state->sample_on_gmax);

          terminal_buffer_lock();
          sprintf(alert_str, "\r\nGmax=%d x=%d, y=%d, z=%d ang=%d\r\n", state->max_g, state->sample_on_gmax.drive_direction,
              state->sample_on_gmax.turn_direction, state->sample_on_gmax.earth_direction, state->hit_angle);
          DisplayMessage(alert_str, 0, false);
          terminal_buffer_release();
          last_print_time = xTaskGetTickCount();
          buzzer_train(5);
        }
      }

      if (device_config.profile_code == PROFILE_LAB) {
        state->accident_state = ACCIDENT_STATE_NONE;
        break;
      }

      if (state->max_g >= (device_config.AccidentG * 10) || driver_behaviour_state.record_triggered) {
        /////////////////////////
        // accident identified //
        /////////////////////////

        FileTransferFailed(true);

        state->last_activity_time = xTaskGetTickCount();
        state->accident_state = ACCIDENT_STATE_IDENTIFIED;
        driver_behaviour_state.record_triggered = false;

        record_trigger(0);
      } else {
        state->accident_state = ACCIDENT_STATE_NONE;
      }
    }

    break;

  case ACCIDENT_STATE_IDENTIFIED:

    driver_behaviour_state.movement_count = 0;

    if (state->accident_sample_count == MIN_SAMPLES_FOR_ACCIDENT) {
      // sending calibrate alert

      state->hit_angle = calculate_accident_hit_angle(&state->sample_on_gmax);

      terminal_buffer_lock();
      sprintf(alert_str + 2, "@?X,%d,%d\r\n", state->max_g, state->hit_angle);
      PostBleAlert(alert_str);
      terminal_buffer_release();

      driver_behaviour_state.accident_count_for_tamper++;

      // beep a buzzer
      if (device_config.buzzer_mode >= BUZZER_MODE_ON)
        buzzer_long(4000);
    }

    state->accident_sample_count++;
    // continue recording
    // ..

    // ~30 seconds delay between accident
    if (state->accident_sample_count >= DELAY_BETWEEN_ACCIDENTS) {

      state->accident_state = ACCIDENT_STATE_NONE;
      DisplayMessage("\r\nListen to accident on\r\n", 0, true);

      if (device_config.buzzer_mode >= BUZZER_MODE_ON)
        buzzer_long(400);
    }
    break;
  }
}