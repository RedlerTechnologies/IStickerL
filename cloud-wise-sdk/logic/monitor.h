#pragma once

#include "hal/hal_data_types.h"

#define DD(s)	((int)((s)[0]-'0')*10+(int)((s)[1]-'0'));

typedef struct
{
  uint8_t year;
  uint8_t month;
  uint8_t day;

  uint8_t hour;
  uint8_t minute;
  uint8_t seconds;
}
Calendar;

void monitor_thread(void *arg);

void InitClock(void);
void SetTimeFromString(uint8_t *date_str, uint8_t *time_str);
uint32_t GetTimeStampFromDate(void);
void SetClockString(uint8_t *buffer);
void PostBleAlert(uint8_t *command_str);
