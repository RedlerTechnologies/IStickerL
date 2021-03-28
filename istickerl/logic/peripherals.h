#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Serial Number String (Device Information Service)
#define SERIAL_NUMBER_WIDTH 9

extern char device_serial_number[SERIAL_NUMBER_WIDTH];

void peripherals_init(void);

uint8_t peripherals_read_battery_level(void);

uint8_t  peripherals_read_temperature(void);
uint16_t peripherals_read_vdd(void);

void peripherals_toggle_leds(void);
