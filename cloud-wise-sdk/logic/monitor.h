#pragma once

#include "hal/hal_data_types.h"

       
typedef struct {
    uint8_t year;
    uint8_t month;
    uint8_t day;

    uint8_t hour;
    uint8_t minute;
    uint8_t seconds;
} Calendar;

void monitor_thread(void *arg);

void     InitClock(void);
void     SetTimeFromString(uint8_t *date_str, uint8_t *time_str);
void     GetSystemTime(Calendar *c);
uint32_t GetTimeStampFromDate(void);
void     SetClockString(uint8_t *buffer);
void     PostBleAlert(uint8_t *command_str);

typedef struct {
    uint32_t last_monitor_time;
    uint16_t  task_bits;
} MonitorState;

#define MONITOR_TEST_TIME 30

#define TASK_MONITOR_NUM 2
#define TASK_MONITOR_BIT_TRACKING 0
#define TASK_MONITOR_BIT_BLE 1

void monitor_task_set(uint16_t task_bit);
bool monitor_task_check(void);
