#pragma once

#include "hal/hal_data_types.h"

#define NUM_OF_PARAMETERS 13

#define PARAM_TYPE_STRING 0
#define PARAM_TYPE_INTEGER 1
#define PARAM_TYPE_REAL 3


typedef struct {
    uint8_t   param_name[16];
    uint32_t *param_address;

    uint16_t param_id;
    uint8_t  max_size;
    uint8_t  param_type;
    uint8_t  counter_id;
    uint8_t  read_only;
} ConfigParameter;

typedef enum {
    COMMAND_BEEP = 0,
    COMMAND_CALIBRATE,
    COMMAND_SLEEP,
    COMMAND_SW_VERSION,
    COMMAND_BUILD,
    COMMAND_DEVICE_ID,
    COMMAND_BLE_ID,
    COMMAND_TIME,
    COMMAND_RESET,
    COMMAND_CLEAR_MEMORY,
    COMMAND_RECORD,
    COMMAND_FILE,
    COMMAND_BLE,
} ECOMMANDS;

typedef struct {
    uint32_t configuration_Changed : 1; // bit 0
    uint32_t ErrorSavingDeviceParams : 1;
    uint32_t WatchdogOccurred : 1;
    uint32_t GPSNotFixed : 1;
    uint32_t GPS_Disconnected : 1;
    uint32_t VersionNeedUpgrade : 1;
    uint32_t NotCalibrated : 1;
    uint32_t Tampered : 1;
    uint32_t GyroConfigurationFailed : 1;
    uint32_t TestMode : 1;
    uint32_t ExternalPower : 1;
    uint32_t Offroad : 1;

    uint32_t Reserved : 20;
} IStickerErrorBits;

bool command_decoder(uint8_t *command_str, uint8_t max_size, uint8_t* result_buffer, uint8_t source);
void delay_sleep(int32_t delay_in_seconds);