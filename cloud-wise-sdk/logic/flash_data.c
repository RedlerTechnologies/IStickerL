#include "flash_data.h"

#include "drivers/flash.h"
#include "logic/serial_comm.h"
#include "nrf_delay.h"
#include "FreeRTOS.h"
#include "semphr.h"

#include <string.h>

#define NRF_LOG_MODULE_NAME flash_data
#define NRF_LOG_LEVEL CLOUD_WISE_DEFAULT_LOG_LEVEL
#include "nrfx_log.h"
NRF_LOG_MODULE_REGISTER();

extern xSemaphoreHandle event_semaphore;

#if (FLASH_TEST_ENABLE)

#define FLASH_TEST_SIZE 64

bool flash_data_test_sector(const uint32_t flash_address)
{
    static uint8_t test_buffer[(FLASH_TEST_SIZE + 16)];

    uint16_t id = flash_read_manufacture_id();

    if (id != 0xef14)
        return false;

    nrf_delay_ms(1);

    memset(test_buffer, 0x00, (FLASH_TEST_SIZE + 16));
    flash_read_buffer(test_buffer, flash_address, FLASH_TEST_SIZE);

    flash_erase_sector(flash_address);

    memset(test_buffer, 0x00, (FLASH_TEST_SIZE + 16));
    flash_read_buffer(test_buffer, flash_address, FLASH_TEST_SIZE);

    if (test_buffer[0x10] != 0xFF)
        return false;

    memset(test_buffer, 0x00, (FLASH_TEST_SIZE + 16));
    for (uint8_t i = 0; i < FLASH_TEST_SIZE; i++)
        test_buffer[i + 4] = i;

    flash_write_buffer(test_buffer, flash_address, FLASH_TEST_SIZE);

    memset(test_buffer, 0x00, (FLASH_TEST_SIZE + 16));
    flash_read_buffer(test_buffer, flash_address, FLASH_TEST_SIZE);

    if (test_buffer[0x10] != 0x0C)
        return false;

    return true;
}

#endif

bool flash_erase_sectors_in_range(uint32_t start_address, uint32_t end_address)
{
    static uint8_t erase_buffer[16];
    uint32_t       address          = start_address;
    uint32_t       bad_sector_count = 0;
    uint16_t       count            = 0;
    bool           flag;

    if (xSemaphoreTake(event_semaphore, 1000) == 0)
        return false;

    while (address < end_address) {
        flag = flash_erase_sector(address);
        if (!flag)
            bad_sector_count++;

        address += FLASH_SECTOR_SIZE;

        // SetMonitorAll();

        if ((count % 10) == 0) {
            sprintf(erase_buffer, "\r\nsector: %d\r\n", count);
            DisplayMessage(erase_buffer, 0);
            // CreateOutput( OUT_RED_LED, 250, 50, 1, OUTPUT_PRIORITY_LOW, 0);
            // CreateOutputX(OUTPUT_X_LED, OUTPUT_SUB_LED_RED, 50, 1, 1);
            // DelaySleep(20, 0);
        }

        count++;
    }

    sprintf(erase_buffer, "\r\nsector: %d\r\n", count);
    DisplayMessage(erase_buffer, 0);

    xSemaphoreGive(event_semaphore);

    return true;
}