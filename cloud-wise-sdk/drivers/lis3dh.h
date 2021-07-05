#pragma once

#include "hal/hal_data_types.h"

bool lis3dh_init(void);
void lis3dh_read_buffer(uint8_t *buffer, uint8_t size, uint8_t reg);

bool lis3dh_configure_idle(void);
bool lis3dh_configure_sleep(void);
bool lis3dh_configure_fifo(void);

bool lis3dh_int_handler(void);

void lis3dh_evt_handler(nrfx_twim_evt_t const *p_event, void *p_context);
