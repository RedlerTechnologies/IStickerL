#pragma once

#include "hal/hal_data_types.h"

#define TIMER_PERIOD 10 // 10ms 100hz acc sampling rate

#define SAMPLE_BUFFER_SIZE 32
#define ACC_MIN_ACCIDENT_VALUE 25
#define MIN_G_FOR_ACCIDENT_EVENT 140

#define MIN_SAMPLES_FOR_ACCIDENT  3

#define ACC_NORMALIZATION_VALUE 1024

#define ACC_MIN_DRIVE_VALUE 25

#define PI 3.1415

typedef enum {
    TRACKING_STATE_WAKEUP = 0,
    TRACKING_STATE_ROUTE = 1,
    TRACKING_STATE_SLEEP = 2,
} TrackingState;

typedef struct {
    short drive_direction;
    short turn_direction;
    short earth_direction;
} AccConvertedSample;

typedef struct {
    signed short X;
    signed short Y;
    signed short Z;
} AccSample;

typedef enum {
    ACCIDENT_STATE_NONE = 0,
    ACCIDENT_STATE_STARTED,
    ACCIDENT_STATE_IDENTIFIED,
    ACCIDENT_STATE_REPORTED,
} AccidentState;

typedef struct {
    TrackingState track_state;

    /////////////////////
    // start algorithm //
    /////////////////////

    uint16_t movement_count;
    uint16_t movement_test_count;

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
    unsigned char  direction_axis;

    ////////////////////////
    // accident algorithm //
    ////////////////////////

    signed short   max_g;
    signed short   sample_in_drive_direction;
    signed short   sample_in_turn_direction;
    unsigned short accident_sample_count;
    signed short   hit_angle;
    unsigned short time_to_sleep_left_in_sec;
    AccidentState  accident_state;

    // on_driving
    unsigned       last_activity_time;
    unsigned short sleep_delay_time;

    // flags
    bool time_synced;
    //bool ble_connected;
    bool new_transfer_protocol;
    bool record_triggered;

    unsigned last_ble_connected_time;

} DriverBehaviourState;

void driver_behaviour_task(void *pvParameter);
void SleepCPU(bool with_memory_retention);