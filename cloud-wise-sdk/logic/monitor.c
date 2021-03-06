#include "monitor.h"

#include "FreeRTOS.h"
#include "app_timer.h"
#include "ble/ble_services_manager.h"
#include "commands.h"
#include "configuration.h"
#include "drivers/buzzer.h"
#include "event_groups.h"
#include "events.h"
#include "flash_data.h"
#include "hal/hal_boards.h"
#include "hal/hal_drivers.h"
#include "logic/peripherals.h"
#include "logic/serial_comm.h"
#include "logic/state_machine.h"
#include "recording.h"
#include "semphr.h"
#include "task.h"
#include "tracking_algorithm.h"

#include <stdlib.h>
#include <string.h>

extern DriverBehaviourState driver_behaviour_state;
extern DeviceConfiguration device_config;
extern IStickerErrorBits error_bits;
extern AccRecord acc_record;
extern uint32_t last_acc_interrupt_time;

APP_TIMER_DEF(m_clock_timer);

#define DD(s) ((int)((s)[0] - '0') * 10 + (int)((s)[1] - '0'));

static Calendar absolute_time __attribute__((section(".non_init")));
static uint32_t clock_counter = 0;
xSemaphoreHandle clock_semaphore;
xSemaphoreHandle watchdog_monitor_semaphore;

ResetData reset_data __attribute__((section(".non_init")));

static MonitorState monitor_state;

uint8_t GetDaysInMonth(uint8_t year, uint8_t month);
void InitClock(void);
void AddSecondsToDate(Calendar *c, uint32_t seconds);
static void BlinkStatusLeds(void);
void print_indicators(void);

void ActivateSoftwareReset(uint8_t reason, uint32_t v1, uint32_t v2, uint32_t v3) {
  reset_data.reason = reason;
  reset_data.v1 = v1;
  reset_data.v2 = v2;
  reset_data.v3 = v3;

  NVIC_SystemReset();
}

static void clock_tick_handler(void *p_context) {
  UNUSED_PARAMETER(p_context);

  clock_counter++;
}

uint32_t getTick(void) {
  uint32_t c;

  taskENTER_CRITICAL();
  c = clock_counter;
  taskEXIT_CRITICAL();

  return c;
}

void PV(uint8_t **buffer, uint8_t val) {
  uint8_t *p = *buffer;
  uint8_t d;

  d = val / 10;
  p[0] = d + '0';

  d = val % 10;
  p[1] = d + '0';

  (*buffer) += 2;
}

void PC(uint8_t **buffer, uint8_t ch) {
  uint8_t *p = *buffer;

  p[0] = ch;
  (*buffer) += 1;
}

void SetClockString(uint8_t *buffer) {
  uint8_t *ptr = buffer;

  xSemaphoreTake(clock_semaphore, portMAX_DELAY);

  PV(&ptr, absolute_time.day);
  PC(&ptr, '-');

  PV(&ptr, absolute_time.month);
  PC(&ptr, '-');

  PV(&ptr, absolute_time.year);
  PC(&ptr, ' ');

  PV(&ptr, absolute_time.hour);
  PC(&ptr, ':');

  PV(&ptr, absolute_time.minute);
  PC(&ptr, ':');

  PV(&ptr, absolute_time.seconds);
  PC(&ptr, ' ');

  xSemaphoreGive(clock_semaphore);
}

void monitor_thread(void *arg) {
  uint32_t prev_current_time = 0;
  uint32_t current_time = 0;
  uint32_t dt = 0;
  bool first = true;

  static uint8_t ble_buffer[16];
  static uint32_t clock_count = 0;

  static uint8_t temperature;
  static uint8_t bat_level;
  static uint16_t vdd;
  static float vdd_float;

  uint32_t duration;

  UNUSED_PARAMETER(arg);

  InitClock();

  prev_current_time = getTick();

  while (1) {

    BlinkStatusLeds();

    vTaskDelay(100);
    current_time = getTick();

    if (current_time == prev_current_time)
      continue;

    // handle overlow in counter later;
    dt = (current_time - prev_current_time);
    prev_current_time = current_time;

    // progress clock here ..
    // ..
    AddSecondsToDate(&absolute_time, dt);

    monitor_task_check();

    close_recording();

#if (FLASH_TEST_ENABLE)

    static uint32_t flash_address = 0x0000;
    bool result = flash_data_test_sector(flash_address);

    driver_behaviour_state.last_activity_time = xTaskGetTickCount();

    terminal_buffer_lock();
    sprintf(alert_str, "\r\nFlash Test: 0x%04X - %d\r\n", flash_address, result);
    DisplayMessageWithNoLock(alert_str, 0);
    terminal_buffer_release();

    flash_address += FLASH_SECTOR_SIZE;

    if (flash_address >= END_OF_FLASH)
      flash_address = 0x0;

    continue;

#endif

    if (driver_behaviour_state.fill_event_flash)
      CreateVersionEvent(false);

    if (device_config.buzzer_mode >= BUZZER_MODE_ON) {
      if ((current_time % 4) == 0) {
        if (driver_behaviour_state.registration_mode)
          buzzer_train(3);
      }
    }

    // checking acc interrupt
    #ifdef RELEASE_COMPILATION
    duration = timeDiff(xTaskGetTickCount(), last_acc_interrupt_time);
    if (duration >= 10000) {
      ActivateSoftwareReset(RESET_ACCELEROMETER_STOP_INTERRUPT, 0, 0, 0);
    }
    #endif

    if ((current_time % 8) == 0 || first) {
      first = false;

      temperature = peripherals_read_temperature();
      bat_level = peripherals_read_battery_level();
      vdd = peripherals_read_vdd();
      vdd_float = ((float)vdd) / 1000;

      duration = timeDiff(xTaskGetTickCount(), driver_behaviour_state.last_activity_time) / 1000;

      terminal_buffer_lock();
      sprintf(alert_str, " - Status: T=%dC, Bat=%d%%, Sleep=%d, VDD=%.2fV, acc=%d\r\n", temperature, bat_level,
          driver_behaviour_state.time_to_sleep_left_in_sec, vdd_float, duration);
      DisplayMessageWithTime(alert_str, 0, false);

      vTaskDelay(10);
      print_indicators();
      terminal_buffer_release();

      // version & measurement as immediate events every ~2 minutes
      if ((current_time % 128) == 0) {
        CreateVersionEvent(true);
        CreateMeasurementEvent(vdd_float, (float)(temperature));
      }

      if ((current_time % 32) == 0 && device_config.min_events_for_tamper && !driver_behaviour_state.tampered) {
        if (driver_behaviour_state.event_count_for_tamper > device_config.min_events_for_tamper) {
          // tampered identified
          set_tamper_mode(LOG_TAMPER_DYNAMIC);
        } else if (driver_behaviour_state.accident_count_for_tamper >= 2) {
          // tampered identified
          set_tamper_mode(LOG_TAMPER_BY_TOO_MANY_ACCIDENTS);
        }

        driver_behaviour_state.event_count_for_tamper = 0;
        driver_behaviour_state.accident_count_for_tamper = 0;
      }

      // search for pending recording files
      duration = timeDiff(xTaskGetTickCount(), acc_record.last_found_record_time) / 1000;
      if (duration >= 30) {
        record_scan_for_new_records(false);
      }

#ifdef BLE_ADVERTISING
      if (ble_services_is_connected())
#endif
      {

#ifdef BLE_ADVERTISING
        ble_services_update_battery_level(bat_level);
#endif

        // send measurements to BLE
        memset(ble_buffer, 0x00, 16);
        memcpy(ble_buffer, (uint8_t *)(&vdd_float), 4);
        ble_buffer[10] = temperature;
        ble_buffer[11] = bat_level;

        // patch: send the number of seconds to sleep
        memcpy(ble_buffer + 8, (uint8_t *)(&driver_behaviour_state.time_to_sleep_left_in_sec), 2);

#ifdef BLE_ADVERTISING
        ble_services_notify_measurement(ble_buffer, 12);
#endif
        // send status and error bit to BLE
        memset(ble_buffer, 0x00, 16);
        ble_buffer[0] = 1; // record type
        error_bits.GPSNotFixed = 1;
        error_bits.GPS_Disconnected = 1;
        error_bits.Tampered = driver_behaviour_state.tampered;
        error_bits.NotCalibrated = !(driver_behaviour_state.calibrated);
        error_bits.NotInsideRoute = !IsInsideRoute();

        // error_bits.NotCalibrated    = 1;
        memcpy(ble_buffer + 6, (uint8_t *)(&error_bits), 4);

#ifdef BLE_ADVERTISING
        ble_services_notify_status(ble_buffer, 16);
#endif

        if (!driver_behaviour_state.time_synced) {
          terminal_buffer_lock();
          sprintf(alert_str + 2, "@?TIME\r\n");
          PostBleAlert(alert_str);
          terminal_buffer_release();
        }
      }
    }
  }
}

void SetTimeFromString(uint8_t *date_str, uint8_t *time_str) {
  Calendar *calendar = &absolute_time;

  calendar->year = DD(date_str + 4);
  calendar->month = DD(date_str + 2);
  calendar->day = DD(date_str);

  calendar->hour = DD(time_str);
  calendar->minute = DD(time_str + 2);
  calendar->seconds = DD(time_str + 4);
}

void GetSystemTime(Calendar *c) { memcpy((void *)c, (uint8_t *)&absolute_time, sizeof(Calendar)); }

void InitClock(void) {
  ret_code_t ret;

  ret = app_timer_create(&m_clock_timer, APP_TIMER_MODE_REPEATED, clock_tick_handler);
  APP_ERROR_CHECK(ret);
  app_timer_start(m_clock_timer, APP_TIMER_TICKS(1000), NULL);
}

static void BlinkStatusLeds(void) {
  static uint32_t last_blink_time = 0;
  uint32_t duration;
  uint8_t pin_index;
  uint8_t i;
  uint8_t blinks = 1;

  duration = timeDiff(xTaskGetTickCount(), last_blink_time) / 1000;

  if (duration >= 8) {
    last_blink_time = xTaskGetTickCount();

    if (ble_services_is_connected())
      pin_index = HAL_LED_GREEN;
    else
      pin_index = HAL_LED_RED;

    if (driver_behaviour_state.track_state == TRACKING_STATE_ROUTE)
      blinks = 2;

    for (i = 0; i < blinks; i++) {

      nrf_gpio_pin_clear(pin_index);
      vTaskDelay(100);
      nrf_gpio_pin_set(pin_index);
      vTaskDelay(100);
    }
  }
}

uint32_t GetTimeStampFromDate(void) {
  Calendar *c = &absolute_time;

  uint32_t timestamp;
  uint32_t days;
  uint8_t i;

  days = c->year * 365;
  days += (c->year - 1) / 4 + 1;

  for (i = 1; i < c->month; i++)
    days += GetDaysInMonth(c->year, i);

  days += c->day - 1;

  timestamp = days * 24 * 3600;
  timestamp += c->hour * 3600;
  timestamp += c->minute * 60;
  timestamp += c->seconds;

  return timestamp;
}

void AddDay(Calendar *c) {

  uint8_t days_in_month = GetDaysInMonth(c->year, c->month);

  if (c->day < days_in_month) {
    c->day++;
  } else {
    c->day = 1;
    if (c->month < 12)
      c->month++;
    else {
      c->month = 1;
      c->year++;
    }
  }
}

void AddHour(Calendar *c) {
  if (c->hour + 1 < 24) {
    c->hour++;
  } else {
    c->hour = 0;
    AddDay(c);
  }
}

void AddMinute(Calendar *c) {
  if (c->minute + 1 < 60) {
    c->minute++;
  } else {
    c->minute = 0;
    AddHour(c);
  }
}

void AddSeconds(Calendar *c) {
  if (c->seconds + 1 < 60) {
    c->seconds++;
  } else {
    c->seconds = 0;
    AddMinute(c);
  }
}

void AddSecondsToDate(Calendar *c, uint32_t seconds) {
  uint8_t day = c->day;

  uint8_t days_in_month;

  while (seconds >= (24 * 3600)) {
    AddDay(c);
    seconds -= (24 * 3600);
  }

  while (seconds >= (3600)) {
    AddHour(c);
    seconds -= (3600);
  }

  while (seconds >= (60)) {
    AddMinute(c);
    seconds -= (60);
  }

  while (seconds > (0)) {
    AddSeconds(c);
    seconds--;
  }
}

uint8_t GetDaysInMonth(uint8_t year, uint8_t month) {
  uint8_t days = 30;

  switch (month) {
  case 1:
  case 3:
  case 5:
  case 7:
  case 8:
  case 10:
  case 12:
    days = 31;
    break;

  case 2:
    if (year % 4)
      days = 28;
    else
      days = 29;
    break;
  }

  return days;
}

void monitor_task_set(uint16_t task_bit) {
  xSemaphoreTake(watchdog_monitor_semaphore, portMAX_DELAY);

  monitor_state.task_bits |= (1 << task_bit);

  xSemaphoreGive(watchdog_monitor_semaphore);
}

void monitor_task_set_all(void) {
  xSemaphoreTake(watchdog_monitor_semaphore, portMAX_DELAY);

  monitor_state.task_bits |= 0xFFFF;

  xSemaphoreGive(watchdog_monitor_semaphore);
}

bool monitor_task_check(void) {
  uint32_t duration;
  uint8_t i;
  bool status = true;
  uint8_t task_id;

#ifdef DISABLE_WATCHDOG
  // state_machine_feed_watchdog();
  return true;
#endif

  xSemaphoreTake(watchdog_monitor_semaphore, portMAX_DELAY);

  duration = timeDiff(xTaskGetTickCount(), monitor_state.last_monitor_time) / 1000;

  if (duration > MONITOR_TEST_TIME) {
    monitor_state.last_monitor_time = xTaskGetTickCount();

    i = 0;
    while (i < TASK_MONITOR_NUM) {

      if ((monitor_state.task_bits & 0x01) == 0) {
        status = false;
        task_id = (i + 1);
        break;
      }

      monitor_state.task_bits = monitor_state.task_bits >> 1;
      i++;
    }

  #ifdef RELEASE_COMPILATION
    if (!status) {
      // force reset on stucked task
      DisplayMessage("\r\nTask is stucked\r\n", 0, true);
      vTaskDelay(200);

      ActivateSoftwareReset(RESET_WATCHDOG, task_id, 0, 0);
    }
    #endif
  }

  state_machine_feed_watchdog();

  xSemaphoreGive(watchdog_monitor_semaphore);

  return status;
}

void print_indicators(void) {
  uint8_t ind_ble;
  uint8_t ind_route;
  uint8_t ind_tamper;
  uint8_t ind_calibrate;
  uint8_t ind_offroad;

  ind_ble = (ble_services_is_connected()) ? '1' : '0';
  ind_route = (IsInsideRoute()) ? '1' : '0';
  ind_tamper = (driver_behaviour_state.tampered) ? '1' : '0';
  ind_calibrate = (driver_behaviour_state.calibratation_saved_in_flash) ? '1' : '0';
  ind_offroad = (error_bits.Offroad) ? '1' : '0';

  memset(alert_str, 0x00, ALERT_BUFFER_SIZE);

  sprintf(alert_str, "BLE=%c, ROUTE=%c, TMP=%c, CAL=%c, OFF=%c\r\n\r\n", ind_ble, ind_route, ind_tamper, ind_calibrate, ind_offroad);
  DisplayMessage(alert_str, 0, false);
}