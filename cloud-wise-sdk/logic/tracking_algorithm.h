#pragma once

#include "hal/hal_data_types.h"

#define TIMER_PERIOD                        10    // 10ms 100hz acc sampling rate

#define SAMPLE_BUFFER_SIZE      32
#define ACC_MIN_ACCIDENT_VALUE  25
#define MIN_G_FOR_ACCIDENT_EVENT  140

#define ACC_NORMALIZATION_VALUE 1024

#define PI 3.1415


typedef struct
{
  short drive_direction;
  short turn_direction;
  short earth_direction;
}
AccConvertedSample;

typedef struct
{
  signed short X;
  signed short Y;
  signed short Z;
}
AccSample;


typedef enum {
    ACCIDENT_STATE_NONE = 0,
    ACCIDENT_STATE_STARTED,
    ACCIDENT_STATE_IDENTIFIED,
    ACCIDENT_STATE_REPORTED,
} AccidentState;


typedef struct
{
  ///////////////////////////
  // calibration algorithm //
  ///////////////////////////

  unsigned char calibrated;

  signed int sum_x;
  signed int sum_y;
  signed int sum_z;

  signed short avg_x;
  signed short avg_y;
  signed short avg_z;

  float angle1;

  unsigned short block_count;
  unsigned char direction_axis;

  ////////////////////////
  // accident algorithm //
  ////////////////////////

  signed short max_g;
  signed short sample_in_drive_direction;
  signed short sample_in_turn_direction;
  unsigned short accident_sample_count;
  AccidentState accident_state;

  // on_driving
  unsigned last_activity_time;
  unsigned short sleep_delay_time;
}
DriverBehaviourState;

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



void driver_behaviour_task(void * pvParameter);