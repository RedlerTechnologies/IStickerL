#pragma once

#include "hal/hal_data_types.h"

#define NUM_OF_PARAMETERS 30

#define PARAM_TYPE_STRING 0
#define PARAM_TYPE_INTEGER 1
#define PARAM_TYPE_REAL 3

#define PARAM_COMMAND 0
#define PARAM_STRING 1
#define PARAM_NUMERIC 2

typedef struct {
    uint8_t   param_name[16];
    uint32_t *param_address;

    uint8_t param_type;
    uint8_t size;

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
    COMMAND_TEST_MODE,
    COMMAND_ACCIDENT_G,
    COMMAND_SAVE,
    COMMAND_DEFAULT_MANUFACTURE,
    COMMAND_SETTINGS,
    COMMAND_DATATX,
    COMMAND_BUZZER_MODE,
    COMMAND_FILTER_Z,
    COMMAND_OFFROAD_G,
    COMMAND_OFFROAD_PER,
    COMMAND_OFFROAD_DIS,
    COMMAND_BUMPER_DIS,
    COMMAND_TAMPER_MIN,
    COMMAND_TAMPER_ANGLE1,
    COMMAND_TAMPER_ANGLE2,
    COMMAND_DRIVER_BEHAVIOUR_CONFIG,
    COMMAND_TAMPER_DIS,
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
    uint32_t NotInsideRoute : 1;

    uint32_t Reserved : 19;
} IStickerErrorBits;

bool command_decoder(uint8_t *command_str, uint8_t max_size, uint8_t *result_buffer, uint8_t source);
// void delay_sleep(int32_t delay_in_seconds);