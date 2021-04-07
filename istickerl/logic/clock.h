#pragma once

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

void InitClock(void);
void SetTimeFromString(uint8_t *date_str, uint8_t *time_str);