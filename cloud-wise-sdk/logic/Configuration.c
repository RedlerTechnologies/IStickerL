#include "hal/hal_boards.h"
#include "hal/hal_drivers.h"
#include "FreeRTOS.h"
#include "task.h"

#include "Configuration.h"
#include "TrackingAlgorithm.h"

#include <string.h>

DeviceConfiguration device_config;

extern DriverBehaviourState driver_behaviour_state;

void LoadConfiguration(void)
{
  device_config.max_sleep_time = 10*60; //120;
  //strcpy ( device_config.DeviceName , "S-KKKK0000001" );

  driver_behaviour_state.sleep_delay_time = device_config.max_sleep_time;
}
