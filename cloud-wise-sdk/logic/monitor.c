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
#include "semphr.h"
#include "task.h"
#include "tracking_algorithm.h"

#include <stdlib.h>
#include <string.h>

extern DriverBehaviourState driver_behaviour_state;
extern IStickerErrorBits    error_bits;

APP_TIMER_DEF(m_clock_timer);

static uint8_t alert_str[48];

// ?????????????????
static Calendar  absolute_time __attribute__((section(".non_init")));
static uint32_t  clock_counter = 0;
xSemaphoreHandle clock_semaphore;

uint8_t GetDaysInMonth(uint8_t year, uint8_t month);
void    InitClock(void);
void    AddSecondsToDate(Calendar *c, uint32_t seconds);

static void clock_tick_handler(void *p_context)
{
    UNUSED_PARAMETER(p_context);

    clock_counter++;
}

uint32_t getTick(void)
{
    uint32_t c;

    taskENTER_CRITICAL();
    c = clock_counter;
    taskEXIT_CRITICAL();

    return c;
}

void PV(uint8_t **buffer, uint8_t val)
{
    uint8_t *p = *buffer;
    uint8_t  d;

    d    = val / 10;
    p[0] = d + '0';

    d    = val % 10;
    p[1] = d + '0';

    (*buffer) += 2;
}

void PC(uint8_t **buffer, uint8_t ch)
{
    uint8_t *p = *buffer;

    p[0] = ch;
    (*buffer) += 1;
}

void SetClockString(uint8_t *buffer)
{
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

void monitor_thread(void *arg)
{
    static uint8_t alert_str[32];

    uint32_t prev_current_time = 0;
    uint32_t current_time      = 0;
    uint32_t dt                = 0;
    bool     first             = true;

    static uint8_t  status_buffer[128];
    static uint8_t  ble_buffer[16];
    static uint32_t clock_count = 0;

    static uint8_t  temperature;
    static uint8_t  bat_level;
    static uint16_t vdd;
    static float    vdd_float;

    uint32_t duration;

    UNUSED_PARAMETER(arg);

    // setting default date/time
    // SetTimeFromString("010120", "000000");

    InitClock();

    prev_current_time = getTick();

    while (1) {

        vTaskDelay(100);
        current_time = getTick();

        if (current_time == prev_current_time)
            continue;

        // handle overlow in ciunter later;
        dt                = (current_time - prev_current_time);
        prev_current_time = current_time;

        // progress clock here ..
        // ..
        AddSecondsToDate(&absolute_time, dt);
        // DisplayMessageWithTime("Clock\r\n", 0);

        state_machine_feed_watchdog();

#if (FLASH_TEST_ENABLE)

        static uint32_t flash_address = 0x0000;
        bool            result        = flash_data_test_sector(flash_address);

        driver_behaviour_state.last_activity_time = xTaskGetTickCount();

        sprintf(alert_str, "\r\nFlash Test: 0x%04X - %d\r\n", flash_address, result);
        DisplayMessage(alert_str, 0);

        flash_address += FLASH_SECTOR_SIZE;

        if (flash_address >= END_OF_FLASH)
            flash_address = 0x0;

        continue;

#endif

        if ((current_time % 16) == 0 || first) {

            first = false;

            temperature = peripherals_read_temperature();
            bat_level   = peripherals_read_battery_level();
            vdd         = peripherals_read_vdd();
            vdd_float   = ((float)vdd) / 1000;

            duration = timeDiff(xTaskGetTickCount(), driver_behaviour_state.last_activity_time) / 1000;
            duration = driver_behaviour_state.sleep_delay_time - duration;

            sprintf(status_buffer, " - Status: T=%dC, Bat=%d%%, Sleep=%d, VDD=%.2fV\r\n\r\n", temperature, bat_level, duration, vdd_float);
            // DisplayMessage(status_buffer, 0);
            DisplayMessageWithTime(status_buffer, 0);

            // NRFX_LOG_INFO("%s Temperature: %dC", __func__, peripherals_read_temperature());

            // uint8_t bat_level = peripherals_read_battery_level();
            // NRFX_LOG_INFO("%s Battery: %u%% VDD mV: %u", __func__, bat_level, peripherals_read_vdd());

            if (is_connected()) {
#ifdef BLE_ADVERTISING
                ble_services_update_battery_level(bat_level);

                // send measurements to BLE
                memset(ble_buffer, 0x00, 16);
                memcpy(ble_buffer, (uint8_t *)(&vdd_float), 4);
                ble_buffer[10] = temperature;
                ble_buffer[11] = bat_level;

                ble_services_notify_measurement(ble_buffer, 12);

                // send status and error bit to BLE
                memset(ble_buffer, 0x00, 16);
                ble_buffer[0]               = 1; // record type
                error_bits.GPSNotFixed      = 1;
                error_bits.GPS_Disconnected = 1;
                // error_bits.NotCalibrated    = 1;
                memcpy(ble_buffer + 6, (uint8_t *)(&error_bits), 4);
                ble_services_notify_status(ble_buffer, 16);
#endif

                if (!driver_behaviour_state.time_synced) {
                    sprintf(alert_str + 2, "@?TIME\r\n");
                    PostBleAlert(alert_str);
                }
            }
        }
    }
}

void SetTimeFromString(uint8_t *date_str, uint8_t *time_str)
{
    Calendar *calendar = &absolute_time;

    calendar->year  = DD(date_str + 4);
    calendar->month = DD(date_str + 2);
    calendar->day   = DD(date_str);

    calendar->hour    = DD(time_str);
    calendar->minute  = DD(time_str + 2);
    calendar->seconds = DD(time_str + 4);
}

void GetSystemTime(Calendar *c) 
{ 
  memcpy((void *)c, (uint8_t*)&absolute_time, sizeof(Calendar)); 
}

void InitClock(void)
{
    ret_code_t ret;

    ret = app_timer_create(&m_clock_timer, APP_TIMER_MODE_REPEATED, clock_tick_handler);
    APP_ERROR_CHECK(ret);
    app_timer_start(m_clock_timer, APP_TIMER_TICKS(1000), NULL);
}

uint32_t GetTimeStampFromDate(void)
{
    Calendar *c = &absolute_time;

    uint32_t timestamp;
    uint32_t days;
    uint8_t  i;

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

void AddDay(Calendar *c)
{

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

void AddHour(Calendar *c)
{
    if (c->hour + 1 < 24) {
        c->hour++;
    } else {
        c->hour = 0;
        AddDay(c);
    }
}

void AddMinute(Calendar *c)
{
    if (c->minute + 1 < 60) {
        c->minute++;
    } else {
        c->minute = 0;
        AddHour(c);
    }
}

void AddSeconds(Calendar *c)
{
    if (c->seconds + 1 < 60) {
        c->seconds++;
    } else {
        c->seconds = 0;
        AddMinute(c);
    }
}

void AddSecondsToDate(Calendar *c, uint32_t seconds)
{
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

uint8_t GetDaysInMonth(uint8_t year, uint8_t month)
{
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