#pragma once

#include "shared_app_config.h"

// <<< Use Configuration Wizard in Context Menu >>>\n

// <h> AM

// <o> BLE_ISTICKERL_BLE_OBSERVER_PRIOs
// <i> Priority with which BLE events are dispatched to the AM Service.

#ifndef BLE_ISTICKERL_BLE_OBSERVER_PRIO
#define BLE_ISTICKERL_BLE_OBSERVER_PRIO 2
#endif

// <o> BLE_ISTICKERL_BLE_LOG_LEVEL  - Default Log level

// <0=> Off
// <1=> Error
// <2=> Warning
// <3=> Info
// <4=> Debug

#ifndef BLE_ISTICKERL_BLE_LOG_LEVEL
#define BLE_ISTICKERL_BLE_LOG_LEVEL NRFX_LOG_LEVEL_INFO
#endif

// </h>

// <<< end of configuration section >>>
