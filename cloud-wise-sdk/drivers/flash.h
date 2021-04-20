#pragma once

#include "hal/hal_data_types.h"

void flash_init(void);
void flash_spi_event_handler(nrfx_spi_evt_t const *p_event, void *p_context);

uint16_t flash_read_manufacture_id(void);

// NOTICE The read data will be started in buffer+4
bool flash_read_buffer(uint8_t *buffer, uint32_t address, uint16_t size);

// NOTICE The write data will be started in buffer+4
bool flash_write_buffer(uint8_t *buffer, uint32_t address, uint16_t size);

bool flash_erase_sector(uint32_t address);

void flash_wait_blocking(void);