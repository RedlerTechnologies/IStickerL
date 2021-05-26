#include "flash_data.h"

#include "FreeRTOS.h"
#include "drivers/flash.h"
#include "logic/decoder.h"
#include "logic/serial_comm.h"
#include "nrf_delay.h"
#include "semphr.h"

#include <string.h>

#define NRF_LOG_MODULE_NAME flash_data
#define NRF_LOG_LEVEL CLOUD_WISE_DEFAULT_LOG_LEVEL
#include "nrfx_log.h"
NRF_LOG_MODULE_REGISTER();

xSemaphoreHandle flash_counter_semaphore;

extern xSemaphoreHandle event_semaphore;
extern char             Flash_buffer[];
extern ScanResult       scan_result;

#define COUNTER_BUFFER_SIZE 32

static uint8_t counter_buffer[COUNTER_BUFFER_SIZE + 4];

static uint32_t counter_address_list[NUM_COUNTERS];

static uint8_t verification_buffer[64];
static uint8_t verification_buffer1[64];

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

bool flash_save_event(uint8_t *data_buffer, uint32_t event_id, uint16_t event_type, uint16_t data_len)
{
    uint32_t address;
    uint32_t value;
    uint32_t start_sector_address;
    uint32_t start_sector_address1;
    uint16_t sector_id = 0;
    uint16_t expected_len;
    uint16_t crcClc, crcClc1;
    uint8_t  need_sector_erase = 0;
    uint8_t  error;
    uint8_t  event_written = 0;
    uint8_t  tries         = 3;

    expected_len = data_len;

    while (!event_written && tries > 0) {
        error = 0;
        tries--;

        // calculate the Flash address of the new event //

        // if (save_in_flash) {
        sector_id = GetSectorID(scan_result.write_marker.flash_address);

        start_sector_address  = GetFlashAddress(sector_id);
        start_sector_address1 = GetFlashAddress(sector_id + 1);

        if (start_sector_address == scan_result.write_marker.flash_address) {
            need_sector_erase = 1;
            address           = start_sector_address;
        } else if ((scan_result.write_marker.flash_address + expected_len) > start_sector_address1) {
            need_sector_erase = 1;
            address           = start_sector_address1;
        } else {
            address = scan_result.write_marker.flash_address;
        }

        CheckFlashAddressLimit(&address);

        // erase the new sector, if needed
        if (need_sector_erase) {
            if (!flash_erase_sector(address)) {
                error = 1;
            }
        }

        // write buffer to flash //

        if (!error) {

            memcpy(verification_buffer, data_buffer, expected_len + 4);
            if (flash_write_buffer(data_buffer, address, expected_len)) {
                flash_read_buffer(verification_buffer1, address, expected_len);
                if (memcmp(verification_buffer + 4, verification_buffer1 + 4, expected_len) != 0) {
                    error = 2;
                }
            } else {
                error = 3;
            }
        }

        if (error) {
            // try another address (next sector) for writing
            // event_flash.next_write_marker. = start_sector_address1;
            scan_result.write_marker.flash_address = start_sector_address1;

            terminal_buffer_lock();
            sprintf(alert_str, "event error: id=%d, type=%d with error=%d\r\n", event_id, event_type, error);
            DisplayMessage(alert_str, 0, false);
            terminal_buffer_release();

        } else {
            event_written = 1;
        }
    }

    if (error) {

        terminal_buffer_lock();
        sprintf(alert_str, "Event Creation Failed: id=%d, type=%d with error=%d\r\n", event_id, event_type, error);
        DisplayMessage(alert_str, 0, false);
        terminal_buffer_release();

        return false;
    } else {

        terminal_buffer_lock();
        sprintf(alert_str, "Event Created: sector=%d, id=%d, type=%d\r\n", sector_id, event_id, event_type);
        DisplayMessage(alert_str, 0, false);
        terminal_buffer_release();

        // if (!system_flags.MinimumTerminalMode)
        //    Display_Message(data_buffer, 0);

        // update write pointer
        // if (save_in_flash) {
        //    scan_result.write_marker.event_id++;

        scan_result.write_marker.flash_address = (address + expected_len);
        // self_test_bits.flash                   = 1;
    }

    return true;
}

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
            DisplayMessage(erase_buffer, 0, true);
            // CreateOutput( OUT_RED_LED, 250, 50, 1, OUTPUT_PRIORITY_LOW, 0);
            // CreateOutputX(OUTPUT_X_LED, OUTPUT_SUB_LED_RED, 50, 1, 1);
            // DelaySleep(20, 0);
        }

        count++;
    }

    sprintf(erase_buffer, "\r\nsector: %d\r\n", count);
    DisplayMessage(erase_buffer, 0, true);

    xSemaphoreGive(event_semaphore);

    return true;
}

uint16_t GetSectorID(uint32_t flash_address)
{
    uint16_t sector_id;

    sector_id = (flash_address - FLASH_EVENTS_START_ADDRESS) / FLASH_SECTOR_SIZE;

    return sector_id;
}

uint32_t GetFlashAddress(uint16_t sector_id)
{
    unsigned flash_address;

    flash_address = FLASH_EVENTS_START_ADDRESS + sector_id * FLASH_SECTOR_SIZE;

    return flash_address;
}

void CheckFlashAddressLimit(uint32_t *flash_address)
{
    if ((*flash_address) >= FLASH_EVENTS_END_ADDRESS) {
        (*flash_address) = FLASH_EVENTS_START_ADDRESS;
    }
}

void CheckSectorLimit(uint16_t *sector_id)
{
    if ((*sector_id) >= NUM_DATA_SECTORS) {
        (*sector_id) = 0;
    }
}

void flash_counter_read(uint8_t counter_idx, uint8_t *value, uint8_t size)
{
    uint32_t flash_address;
    uint16_t i, j, k;
    bool     empty;

    xSemaphoreTake(flash_counter_semaphore, portMAX_DELAY);

    if (counter_address_list[counter_idx] > 0) {
        flash_address = counter_address_list[counter_idx];
        flash_read_buffer(counter_buffer, flash_address, size);
        memcpy((uint8_t *)value, &counter_buffer[4], size);
    } else {

        flash_address = FLASH_COUNTER_START_ADDRESS + counter_idx * FLASH_SECTOR_SIZE;
        i             = 0;

        counter_address_list[counter_idx] = flash_address;
        memset(value, 0xFF, size);

        while (i < FLASH_SECTOR_SIZE) {

            flash_read_buffer(counter_buffer, flash_address, COUNTER_BUFFER_SIZE);

            k = 0;
            while (k < COUNTER_BUFFER_SIZE) {

                empty = true;

                for (j = 0; j < size; j++) {
                    if (counter_buffer[4 + k] != 0xFF) {
                        empty = false;
                        break;
                    }
                }

                if (empty) {
                    xSemaphoreGive(flash_counter_semaphore);
                    return;
                }

                memcpy((uint8_t *)value, &counter_buffer[4 + k], size);
                counter_address_list[counter_idx] = flash_address + k;

                k += size;
            }

            i += COUNTER_BUFFER_SIZE;
            flash_address += COUNTER_BUFFER_SIZE;
        }
    }

    xSemaphoreGive(flash_counter_semaphore);
}

void flash_counter_write(uint8_t counter_idx, uint8_t *new_value, uint8_t size)
{
    uint32_t flash_address;
    uint32_t old_value;
    uint16_t sector_id, sector_id1;

    xSemaphoreTake(flash_counter_semaphore, portMAX_DELAY);

    flash_address = counter_address_list[counter_idx];
    flash_read_buffer(counter_buffer, flash_address, size);
    memcpy((uint8_t *)&old_value, &counter_buffer[4], size);

    sector_id  = GetSectorID(flash_address);
    sector_id1 = GetSectorID(flash_address + size);

    if (sector_id != sector_id1) {
        flash_address = GetFlashAddress(sector_id);
        flash_erase_sector(flash_address);
        // what to do if erase fails....
    } else {
        if (old_value != 0xFFFFFFFF)
            flash_address += size;
    }

    memcpy(counter_buffer + 4, new_value, size);
    flash_write_buffer(counter_buffer, flash_address, size);
    counter_address_list[counter_idx] = flash_address;

    xSemaphoreGive(flash_counter_semaphore);
}

bool IsDBInitialized(void)
{
    bool ret;

    ret = (scan_result.write_marker.event_id > 0);

    return ret;
}