#pragma once

#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "sdk_config.h"

#include <stdbool.h>
#include <stdint.h>

#include "ble_istickerl.h"


typedef struct {
    uint8_t message[COMMAND_CHAR_MAX_LEN];
    uint8_t size;
} BleMessage;


void ble_thread(void *pvParameters);
void init_ble_task(void);
bool PostBleCommand(uint8_t *command_str, uint8_t size);
void PostBleAlert(uint8_t *command_str);
