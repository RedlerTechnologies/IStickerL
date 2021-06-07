#pragma once

#include "hal/hal_data_types.h"

#define ACC_TABLE_DRIVER_SIZE 17
#define ACC_TABLE_SLEEP_SIZE 2

bool lis3dh_init(void);
void lis3dh_write_reg(uint8_t reg, uint8_t value);
uint8_t lis3dh_read_reg(uint8_t reg);

void lis3dh_read_buffer( uint8_t* buffer, uint8_t size, uint8_t reg);

bool configure_acc( unsigned char* table, unsigned char table_size);