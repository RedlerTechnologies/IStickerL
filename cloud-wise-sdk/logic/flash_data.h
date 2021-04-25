#pragma once

#include <stdbool.h>
#include <stdint.h>

bool flash_data_test_sector(const uint32_t flash_address);
bool flash_erase_sectors_in_range(uint32_t start_address, uint32_t end_address);


#define FLASH_SECTOR_SIZE 0x1000
#define END_OF_FLASH 0x200000