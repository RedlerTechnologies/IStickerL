#include "configuration.h"

#include "FreeRTOS.h"
#include "hal/hal_boards.h"
#include "hal/hal_drivers.h"
#include "task.h"
#include "tracking_algorithm.h"

#include <string.h>

DeviceConfiguration device_config;

extern DriverBehaviourState driver_behaviour_state;

void LoadConfiguration(void)
{
    device_config.max_sleep_time = 10 * 60; // 120;
    // strcpy ( device_config.DeviceName , "S-KKKK0000001" );

    driver_behaviour_state.sleep_delay_time = device_config.max_sleep_time;
}