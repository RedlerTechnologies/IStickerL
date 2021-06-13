#pragma once

#include "hal/hal_data_types.h"

#define ACC_TABLE_DRIVER_SIZE 15
#define ACC_TABLE_SLEEP_SIZE 3 

bool    lis3dh_init(void);
void    lis3dh_read_buffer(uint8_t *buffer, uint8_t size, uint8_t reg);
bool    lis3dh_configure(uint8_t *table, uint8_t table_size);
void    lis3dh_evt_handler(nrfx_twim_evt_t const *p_event, void *p_context);