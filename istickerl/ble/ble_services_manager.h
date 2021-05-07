#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BLE_EVENTS_ENTER_FAST_ADV 0x00000001
#define BLE_EVENTS_ENTER_SLOW_ADV 0x00000002
#define BLE_EVENTS_ENTER_NO_ADV 0x00000004
#define BLE_EVENTS_SERVER_TIMEOUT 0x00000008
#define BLE_EVENTS_CLIENT_TIMEOUT 0x00000010
#define BLE_EVENTS_CONNECTED 0x00000020
#define BLE_EVENTS_DISCONNECTED 0x00000040

void ble_services_init_0(void);
void ble_services_init(void);
void ble_services_advertising_start(void);

bool ble_services_is_connected(void);
void ble_services_disconnect(void);

void ble_services_update_battery_level(uint8_t battery_level);

bool ble_services_notify_command(uint8_t *const data, size_t length);
bool ble_services_notify_measurement(uint8_t *const data, size_t length);
bool ble_services_notify_acc(uint8_t *const data, size_t length);
bool ble_services_notify_status(uint8_t *const data, size_t length);
bool ble_services_notify_event(uint8_t *const data, size_t length);
bool ble_services_notify_file_transfer(uint8_t *const data, size_t length);

bool ble_services_update_acc(uint8_t *const data, size_t length);
bool ble_services_update_status(uint8_t *const data, size_t length);
bool ble_services_update_measurement(uint8_t *const data, size_t length);
bool ble_services_update_event(uint8_t *const data, size_t length);
bool ble_services_update_file_transfer(uint8_t *const data, size_t length);

void set_device_name(void);