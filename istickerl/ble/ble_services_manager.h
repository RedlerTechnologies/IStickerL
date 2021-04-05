#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void ble_services_init_0(void);
void ble_services_init(void);
void ble_services_advertising_start(void);

void ble_services_update_battery_level(uint8_t battery_level);

bool ble_services_update_command(uint8_t *const data, size_t length);
bool ble_services_update_acc(uint8_t *const data, size_t length);
bool ble_services_update_status(uint8_t *const data, size_t length);
bool ble_services_update_measurement(uint8_t *const data, size_t length);
bool ble_services_update_event(uint8_t *const data, size_t length);