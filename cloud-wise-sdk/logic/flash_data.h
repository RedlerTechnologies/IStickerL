#pragma once

#include "recording.h"

#include <stdbool.h>
#include <stdint.h>

bool flash_data_test_sector(const uint32_t flash_address);
bool flash_erase_sectors_in_range(uint32_t start_address, uint32_t end_address);
void CheckFlashAddressLimit(uint32_t *flash_address);
void CheckSectorLimit(uint16_t *sector_id);
bool flash_save_event(uint8_t *data_buffer, uint32_t event_id, uint16_t event_type, uint16_t data_len);
bool IsDBInitialized(void);

void flash_counter_read(uint8_t counter_idx, uint8_t *value, uint8_t size);
void flash_counter_write(uint8_t counter_idx, uint8_t *value, uint8_t size);

uint16_t GetSectorID(uint32_t flash_address);
uint32_t GetFlashAddress(uint16_t sector_id);

#define FLASH_SECTOR_SIZE 0x1000
#define END_OF_FLASH 0x200000

#define FLASH_DEVICE_PARAMS_ADDRESS 0x0000
#define FLASH_APP_PARAMS_ADDRESS (FLASH_SECTOR_SIZE)

// flash counter definishions: //
#define FLASH_COUNTER_START_ADDRESS 0x2000

#define NUM_COUNTERS 2
#define FLASH_COUNTER_IDX_LAST_SENT_EVENT_ID 0
#define FLASH_COUNTER_IDX_LAST_SENT_EVENT_ADDRESS 1
// end of flash counter defintions //

#define FLASH_EVENTS_START_ADDRESS 0x8000

#define FLASH_EVENTS_END_ADDRESS FLASH_RECORDS_START_ADDRESS
#define NUM_DATA_SECTORS (FLASH_EVENTS_END_ADDRESS - FLASH_EVENTS_START_ADDRESS) / FLASH_SECTOR_SIZE