#include "recording.h"

#include "FreeRTOS.h"
#include "ble_file_transfer.h"
#include "drivers/buzzer.h"
#include "drivers/flash.h"
#include "events.h"
#include "flash_data.h"
#include "hal/hal_boards.h"
#include "logic/serial_comm.h"
#include "monitor.h"
#include "nrf_delay.h"
#include "semphr.h"
#include "timers.h"
#include "tracking_algorithm.h"
#include "decoder.h"

#include <string.h>

#define NRF_LOG_MODULE_NAME decoder
#define NRF_LOG_LEVEL CLOUD_WISE_DEFAULT_LOG_LEVEL
#include "nrfx_log.h"
NRF_LOG_MODULE_REGISTER();

