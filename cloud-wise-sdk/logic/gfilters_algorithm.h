#pragma once

#include "tracking_algorithm.h"

#define MAX_EVENT_DURATION 3000 // 3 seconds

#define GFILTER_NUM 4
#define MAX_EVENTS_IN_QUEUE 5
#define TIME_EVENT_IN_QUEUE 5
#define DELAY_BETWEEN_GFILTER_EVENT_MSEC 2000

// table of driver behaviour thresholds
// g in handredth of g
// duration in msec
#define ACCELERATION_MIN_G 20
#define ACCELERATION_MIN_DUR 100
#define ACCELERATION_MIN_MED_G 35
#define ACCELERATION_MIN_MED_DUR 200
#define ACCELERATION_MIN_HIGH_G 55
#define ACCELERATION_MIN_HIGH_DUR 200

#define BRAKES_MIN_G 20
#define BRAKES_MIN_DUR 100
#define BRAKES_MIN_MED_G 55
#define BRAKES_MIN_MED_DUR 200
#define BRAKES_MIN_HIGH_G 75
#define BRAKES_MIN_HIGH_DUR 350

#define TURN_MIN_G 20
#define TURN_MIN_DUR 100
#define TURN_MIN_MED_G 50
#define TURN_MIN_MED_DUR 250
#define TURN_MIN_HIGH_G 75
#define TURN_MIN_HIGH_DUR 350

#define GFILTER_ACCELERATION 0
#define GFILTER_BRAKES 1
#define GFILTER_TURN_LEFT 2
#define GFILTER_TURN_RIGHT 3

#define GFILTER_AXIS_X 0
#define GFILTER_AXIS_Y 1
#define GFILTER_AXIS_Z 2

#define GFILTER_AXIS_STATE_NONE 0
#define GFILTER_AXIS_STATE_IDENTIFIED 1
#define GFILTER_AXIS_STATE_COMPLETED 2
#define GFILTER_AXIS_STATE_DELAY 3
#define GFILTER_AXIS_STATE_COMPLETED2 4
#define GFILTER_AXIS_STATE_DELAY2 5 

#define DRIVER_BEHAVIOR_EVENT_BRAKES 0
#define DRIVER_BEHAVIOR_EVENT_ACCEL 1
#define DRIVER_BEHAVIOR_SLALUM 2
#define DRIVER_BEHAVIOR_SHARP_TURN 3
#define DRIVER_BEHAVIOR_BRAKE_INSIDE_TURN 4
#define DRIVER_BEHAVIOR_SHARP_TURN_RIGHT 30

typedef struct {
    uint8_t code;
    uint8_t axis;
    uint8_t positive;

    int16_t  min_g;         // units: 1/100 g
    uint16_t min_duration;  // units in msec
    int16_t  min_g2;        // units: 1/100 g
    uint16_t min_duration2; // units in msec
    int16_t  min_g3;        // units: 1/100 g
    uint16_t min_duration3; // units in msec

    uint8_t name[8];
} GFilterConfig;

typedef struct {
    uint32_t energy;
    uint32_t timestamp;
    uint32_t event_time;
    uint32_t duration_count;
    uint32_t max_g;
    uint8_t  state;
    uint8_t  severity;
    uint8_t  index;
} GFilterState;

void Process_GFilters(AccConvertedSample *samples);
void gfilter_init(void);