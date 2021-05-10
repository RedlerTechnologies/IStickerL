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

typedef struct {
    uint32_t v1;
    uint32_t v2;
    uint32_t v3;
    uint8_t  reason;
} ResetData;

void monitor_thread(void *arg);

void     InitClock(void);
void     SetTimeFromString(uint8_t *date_str, uint8_t *time_str);
void     GetSystemTime(Calendar *c);
uint32_t GetTimeStampFromDate(void);
void     SetClockString(uint8_t *buffer);
void     PostBleAlert(uint8_t *command_str);
void     ActivateSoftwareReset(uint8_t reason, uint32_t v1, uint32_t v2, uint32_t v3);

typedef struct {
    uint32_t last_monitor_time;
    uint16_t task_bits;
} MonitorState;

#define RESET_COMMAND 1
#define RESET_WATCHDOG 2
#define RESET_HARD_FAULT 3
#define RESET_AFTER_SLEEP 4
#define RESET_POWER_OFF 5

#define MONITOR_TEST_TIME 30

#define TASK_MONITOR_NUM 3
#define TASK_MONITOR_BIT_TRACKING 0
#define TASK_MONITOR_BIT_BLE 1
#define TASK_MONITOR_BIT_UART_RX 2

#define ALERT_BUFFER_SIZE (256+16)

void monitor_task_set(uint16_t task_bit);
bool monitor_task_check(void);