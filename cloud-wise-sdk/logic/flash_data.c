#include "drivers/flash.h"
#include "nrf_delay.h"

#include <string.h>

#define NRF_LOG_MODULE_NAME cloud_wise_sdk_logic_flash_data
#define NRF_LOG_LEVEL CLOUD_WISE_DEFAULT_LOG_LEVEL
#include "nrfx_log.h"
NRF_LOG_MODULE_REGISTER();

bool flash_data_test_sector(const uint32_t flash_address)
{
    static uint8_t test_buffer[512];

    uint16_t id = flash_read_manufacture_id();

    if (id != 0xef14)
        return false;

    flash_wait_blocking();

    nrf_delay_ms(1);

    /*
        WriteLock(true);
        WaitForFlash();
        WriteLock(false);
        WaitForFlash();
        */

    memset(test_buffer, 0x00, 512);
    flash_read_buffer(test_buffer, flash_address, 128);

    flash_erase_sector(flash_address);

    memset(test_buffer, 0x00, 512);
    flash_read_buffer(test_buffer, flash_address, 128);

    if (test_buffer[0x10] != 0xFF)
        return false;

    memset(test_buffer, 0x00, 512);
    for (uint8_t i = 0; i < 64; i++)
        test_buffer[i + 4] = i;

    flash_write_buffer(test_buffer, flash_address, 64);

    memset(test_buffer, 0x00, 512);
    flash_read_buffer(test_buffer, flash_address, 128);

    if (test_buffer[0x10] != 0x0C)
        return false;

    return true;
}