#pragma once

#include "hal/hal_data_types.h"

#define TIMER_PERIOD 10 // 10ms 100hz acc sampling rate

#define SAMPLE_BUFFER_SIZE 32
#define ACC_MIN_ACCIDENT_VALUE 25
#define MIN_G_FOR_ACCIDENT_EVENT 14 // 25 // ???????????

#define MIN_SAMPLES_FOR_ACCIDENT 3

#define ACC_NORMALIZATION_VALUE 1024

#define ACC_MIN_DRIVE_VALUE 25

#define DELAY_BETWEEN_ACCIDENTS 2000 // ~30 seconds

#define MIN_ENERY_FOR_START_ROUTE 550
#define MIN_ENERY_FOR_CONTINUE_ROUTE 650

#define SLEEP_TIMEOUT_ON_ROUTE_BLE_CONNECTED (10 * 60)
#define SLEEP_TIMEOUT_ON_ROUTE_BLE_DISCONNECTED (2 * 60)
#define SLEEP_TIMEOUT_ON_WAKEUP_BLE_CONNECTED (10 * 60)
#define SLEEP_TIMEOUT_ON_WAKEUP_BLE_DISCONNECTED (45)

#define PI 3.1415

typedef enum {
    TRACKING_STATE_WAKEUP = 0,
    TRACKING_STATE_ROUTE  = 1,
    TRACKING_STATE_SLEEP  = 2,
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

typedef struct {
    AccSample avg_value;
    uint8_t   axis;
} CalibratedValue;

typedef enum {
    ACCIDENT_STATE_NONE = 0,
    ACCIDENT_STATE_STARTED,
    ACCIDENT_STATE_IDENTIFIED,
    ACCIDENT_STATE_REPORTED,
} AccidentState;

#define TEST_MODE_TRIGGER_ACCIDENT 8
#define PRINT_SIGNAL_MODE_ENERGY 9
#define TEST_MODE_PRINT_SENT_EVENT

#define PRINT_SIGNAL_MODE_GX 11
#define PRINT_SIGNAL_MODE_GY 12
#define TEST_MODE_PRINT_ALL_EVENTS 20
#define TEST_MODE_PRINT_ALL_EVENTS_AND_DATA 21
#define TEST_MODE_PRINT_SENT_EVENT_MODE 30
#define TEST_MODE_DROP_NEW_EVENTS 31
#define TEST_MODE_FILL_EVENT_FLASH 32
#define TEST_MODE_PRINT_SENT_DATA_MODE 33

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

    CalibratedValue calibrated_value;

    unsigned char calibrated;
    unsigned char calibratation_saved_in_flash;
    unsigned char store_calibration;

    signed int sum_x;
    signed int sum_y;
    signed int sum_z;

    float angle1;

    unsigned short block_count;

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

    unsigned short acc_int_counter;

    // flags
    bool time_synced;
    bool new_transfer_protocol;
    bool record_triggered;
    bool tampered;
    bool manual_delayed;
    bool print_event_data;
    bool print_sent_event_id_mode;
    bool in_event_transfer_process;
    bool block_new_events;
    bool fill_event_flash;
    bool print_sent_event_data_mode;

    uint8_t print_signal_mode;

    unsigned last_ble_connected_time;
    unsigned stop_advertising_time;

    signed       energy;
    signed short Gx;
    signed short Gy;

} DriverBehaviourState;

void driver_behaviour_task(void *pvParameter);
void SleepCPU(bool with_memory_retention);
void clear_calibration(void);
void copy_calibration(void);

bool IsDeviceMoved(unsigned moved_in_last_seconds);
void set_sleep_timeout(uint16_t value);
void set_sleep_timeout_on_ble(void);