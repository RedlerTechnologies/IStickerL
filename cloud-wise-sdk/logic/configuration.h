#pragma once

#include "hal/hal_data_types.h"

typedef struct
{
  unsigned short max_sleep_time;
  unsigned char DeviceName[16]; // BLEID
  unsigned char DeviceID[28];
  
  unsigned char AccidentG;
}
DeviceConfiguration;


void LoadConfiguration(void);
