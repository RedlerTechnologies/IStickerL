#pragma once

#include "hal/hal_data_types.h"
#include "tracking_algorithm.h"

typedef struct {
    // 0x0000
    unsigned char DeviceName[16]; // BLEID

    // 0x0010
    unsigned char DeviceID[28];

    unsigned char AccidentG;
    unsigned char buzzer_mode;
    unsigned char reservered3;

    // 0x0030
    unsigned short max_sleep_time;
    unsigned short filter_z;
    unsigned short reservered11;
    unsigned short reservered12;
    unsigned short reservered13;
    unsigned short reservered14;
    unsigned short reservered15;
    unsigned short reservered16;

    // 0x0040
    CalibratedValue calibrate_value; // size 8 bytes
    unsigned char   reserved20[8];

    // 0x0050
    unsigned char reserved21[16];

    // 0x0060
    unsigned char reserved22[16];

    // 0x0070
    unsigned char reserved23[16];

    // 0x0080
    unsigned char reserved24[16];

    // 0x0090
    unsigned char reserved25[16];

    // 0x00A0
    unsigned char reserved26[16];

    // 0x00B0
    unsigned char reserved27[16];

    // 0x00C0
    unsigned char reserved28[16];

    // 0x00D0
    unsigned char reserved29[16];

    // 0x00E0
    unsigned char reserved30[16];

    // 0x0030
    unsigned short reservered40;
    unsigned short reservered41;
    unsigned short reservered42;
    unsigned short reservered43;
    unsigned short reservered44;
    unsigned short reservered45;
    unsigned short reservered47;
    unsigned short crc;

} DeviceConfiguration;

void LoadConfiguration(void);
void SetManufactureDefault(void);
void SaveConfiguration(bool force);