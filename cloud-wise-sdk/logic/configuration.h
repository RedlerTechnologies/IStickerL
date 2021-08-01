#pragma once

#include "gfilters_algorithm.h"
#include "hal/hal_data_types.h"
#include "tracking_algorithm.h"

#define PROFILE_PRODUCTION 0
#define PROFILE_PILOT 1
#define PROFILE_EXPERIMENT 2
#define PROFILE_LAB 3

typedef struct {
  uint32_t reserved_bit_00 : 1;
  uint32_t bumper_dis : 1;
  uint32_t reserved_bit_02 : 1;
  uint32_t reserved_bit_03 : 1;
  uint32_t reserved_bit_04 : 1;
  uint32_t reserved_bit_05 : 1;
  uint32_t reserved_bit_06 : 1;
  uint32_t reserved_bit_07 : 1;
  uint32_t reserved_bit_08 : 1;
  uint32_t reserved_bit_09 : 1;
  uint32_t reserved_bit_10 : 1;
  uint32_t reserved_bit_11 : 1;
  uint32_t reserved_bit_12 : 1;
  uint32_t offroad_disabled : 1;
  uint32_t tamper_disabled : 1;
  uint32_t reserved_bit_15 : 1;
  uint32_t reserved_bit_16 : 1;
  uint32_t reserved_bit_17 : 1;
  uint32_t reserved_bit_18 : 1;
  uint32_t reserved_bit_19 : 1;
  uint32_t reserved_bit_20 : 1;
  uint32_t reserved_bit_21 : 1;
  uint32_t reserved_bit_22 : 1;
  uint32_t reserved_bit_23 : 1;
  uint32_t reserved_bit_24 : 1;
  uint32_t reserved_bit_25 : 1;
  uint32_t reserved_bit_26 : 1;
  uint32_t reserved_bit_27 : 1;
  uint32_t reserved_bit_28 : 1;
  uint32_t reserved_bit_29 : 1;
  uint32_t reserved_bit_30 : 1;
  uint32_t reserved_bit_31 : 1;
} ConfigFlags;

typedef struct {
  // 0x0000
  unsigned char DeviceName[16]; // BLEID

  // 0x0010
  unsigned char DeviceID[28];

  unsigned char AccidentG;
  unsigned char buzzer_mode;
  unsigned char profile_code;

  // 0x0030
  unsigned short offroad_g;
  unsigned short filter_z;
  unsigned char offroad_per;
  unsigned char min_events_for_tamper;
  unsigned char tamper_angle1;
  unsigned char tamper_angle2;
  unsigned short reservered13;
  unsigned short reservered14;
  ConfigFlags config_flags;

  // 0x0040
  CalibratedValue calibrate_value; // size 12 bytes
  unsigned char reserved20[4];

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
  unsigned short dr_bh_gvalues[NUM_DRIVER_EVENT_TYPES][3];
  unsigned char reserved_50;
  unsigned char reserved_52;

  // 0x00E0
  unsigned char dr_bh_durations[NUM_DRIVER_EVENT_TYPES][3];
  unsigned char reserved_51;

  // 0x00E0
  //unsigned char reserved30[16];

  // 0x00F0
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
void SetManufactureDefault(uint8_t profile_num);
void SaveConfiguration(bool force);