#include "flash.h"

#include "FreeRTOS.h"
#include "hal/hal.h"
#include "hal/hal_drivers.h"
#include "nrf_log_ctrl.h"
#include "semphr.h"

#define NRF_LOG_MODULE_NAME cloud_wise_sdk_drivers_flash
#define NRF_LOG_LEVEL CLOUD_WISE_DEFAULT_LOG_LEVEL
#include "nrfx_log.h"
NRF_LOG_MODULE_REGISTER();

#define MANUFACTURER_ID 0x90
#define STATUS_REG 0x05
#define READ_DATA 0x03
#define WRITE_ENABLE 0x06
#define WRITE_DISABLE 0x04
#define PAGE_PROGRAM 0x02
#define ERASE_SECTOR 0x20

static volatile bool m_spi_xfer_done;

static void set_write_lock(bool is_enabled);
static void flash_wait_blocking(void);

xSemaphoreHandle flash_semaphore;

void flash_spi_event_handler(nrfx_spi_evt_t const *p_event, void *p_context)
{
    m_spi_xfer_done = true;
    NRF_LOG_INFO("Transfer completed.");

    // if (m_rx_buf[0] != 0) {
    //  NRF_LOG_INFO(" Received:");
    //  NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
    //}
}

void flash_init(void)
{
    //
}

uint16_t flash_read_manufacture_id(void)
{
    uint8_t tx_buf[5];
    uint8_t rx_buf[6];

    uint16_t ret;

    xSemaphoreTake(flash_semaphore, portMAX_DELAY);

    memset(tx_buf, 0x00, 5);
    memset(rx_buf, 0x00, 6);

    m_spi_xfer_done = false;

    tx_buf[0] = MANUFACTURER_ID;

    nrfx_spi_xfer_desc_t xfer = NRFX_SPI_XFER_TRX(tx_buf, 5, rx_buf, 6);

    APP_ERROR_CHECK(nrfx_spi_xfer(hal_flash_spi, &xfer, 0));

    while (!m_spi_xfer_done) {
        __WFE();
    }

    ret = rx_buf[4] << 8;
    ret |= rx_buf[5];

    xSemaphoreGive(flash_semaphore);

    return ret;
}

// NOTICE The read data will be started in buffer+4
bool flash_read_buffer(uint8_t *buffer, uint32_t address, uint16_t size)
{
    xSemaphoreTake(flash_semaphore, portMAX_DELAY);

    memset(buffer, 0x00, (size + 4));

    buffer[0] = READ_DATA;
    buffer[1] = ((address >> 16) & 0xFF);
    buffer[2] = ((address >> 8) & 0xFF);
    buffer[3] = ((address)&0xFF);

    m_spi_xfer_done = false;

    nrfx_spi_xfer_desc_t xfer = NRFX_SPI_XFER_TRX(buffer, size + 4, buffer, size + 4);

    APP_ERROR_CHECK(nrfx_spi_xfer(hal_flash_spi, &xfer, 0));

    while (!m_spi_xfer_done) {
        __WFE();
    }

    xSemaphoreGive(flash_semaphore);

    return true;
}

// the write data will be started in buffer+4
static bool flash_write_buffer_internal(uint8_t *buffer, uint32_t address, uint16_t size)
{
    ret_code_t ret;

    // xSemaphoreTake(flash_semaphore, portMAX_DELAY);

    set_write_lock(true);

    buffer[0] = PAGE_PROGRAM;
    buffer[1] = ((address >> 16) & 0xFF);
    buffer[2] = ((address >> 8) & 0xFF);
    buffer[3] = ((address)&0xFF);

    m_spi_xfer_done = false;

    nrfx_spi_xfer_desc_t xfer = NRFX_SPI_XFER_TX(buffer, size + 4);

    APP_ERROR_CHECK(nrfx_spi_xfer(hal_flash_spi, &xfer, 0));

    while (!m_spi_xfer_done) {
        __WFE();
    }

    flash_wait_blocking();

    set_write_lock(false);

    // xSemaphoreGive(flash_semaphore);

    return true;
}

bool flash_write_buffer(uint8_t *buffer, uint32_t address, uint16_t size)
{
    int16_t    byte_left;
    ret_code_t ret;

    xSemaphoreTake(flash_semaphore, portMAX_DELAY);

    byte_left = size;

    while (byte_left > 0) {

        if (((address % 256) + byte_left) > 256) {
            size = 256 - (address % 256);
        } else {
            size = byte_left;
        }

        flash_write_buffer_internal(buffer, address, size);

        byte_left -= size;
        address += size;
        buffer += size;
    }

    xSemaphoreGive(flash_semaphore);

    return true;
}

/*
// the write data will be started in buffer+4
bool flash_write_buffer(uint8_t *buffer, uint32_t start_address, uint16_t size)
{
    uint32_t   address;
    int16_t   byte_left;
    ret_code_t ret;

    xSemaphoreTake(flash_semaphore, portMAX_DELAY);


    byte_left = size;

    while (byte_left > 0) {

        set_write_lock(true);

        if (((start_address % 256) + byte_left) > 256) {
            size = 256 - (start_address % 256);
        } else {
            size = byte_left;
        }

        address = start_address;

        buffer[0] = PAGE_PROGRAM;
        buffer[1] = ((address >> 16) & 0xFF);
        buffer[2] = ((address >> 8) & 0xFF);
        buffer[3] = ((address)&0xFF);

        m_spi_xfer_done = false;

        nrfx_spi_xfer_desc_t xfer = NRFX_SPI_XFER_TX(buffer, size + 4);

        APP_ERROR_CHECK(nrfx_spi_xfer(hal_flash_spi, &xfer, 0));

        while (!m_spi_xfer_done) {
            __WFE();
        }

        byte_left -= size;
        start_address += size;
        buffer += size;

        flash_wait_blocking();

        set_write_lock(false);
    }


    xSemaphoreGive(flash_semaphore);

    return true;
}
*/

bool flash_erase_sector(uint32_t address)
{

    uint8_t    buffer[4];
    ret_code_t ret;

    xSemaphoreTake(flash_semaphore, portMAX_DELAY);

    set_write_lock(true);

    buffer[0] = ERASE_SECTOR;
    buffer[1] = ((address >> 16) & 0xFF);
    buffer[2] = ((address >> 8) & 0xFF);
    buffer[3] = ((address)&0xFF);

    m_spi_xfer_done = false;

    nrfx_spi_xfer_desc_t xfer = NRFX_SPI_XFER_TX(buffer, 4);

    APP_ERROR_CHECK(nrfx_spi_xfer(hal_flash_spi, &xfer, 0));

    while (!m_spi_xfer_done) {
        __WFE();
    }

    set_write_lock(false);

    flash_wait_blocking();

    xSemaphoreGive(flash_semaphore);

    return true;
}

static void flash_wait_blocking(void)
{
    uint8_t tx_buf[1] = {STATUS_REG};
    uint8_t rx_buf[2];

    bool busy = 1;

    nrfx_spi_xfer_desc_t xfer = NRFX_SPI_XFER_TRX(tx_buf, 1, rx_buf, 2);

    while (busy) {

        memset(rx_buf, 0x00, 2);
        m_spi_xfer_done = false;

        APP_ERROR_CHECK(nrfx_spi_xfer(hal_flash_spi, &xfer, 0));

        while (!m_spi_xfer_done) {
            __WFE();
        }

        if ((rx_buf[1] & 0x01) == 0x00)
            busy = 0;
    }
}

static void set_write_lock(bool is_enable)
{
    uint8_t buf[1] = {is_enable ? WRITE_ENABLE : WRITE_DISABLE};

    m_spi_xfer_done = false;

    nrfx_spi_xfer_desc_t xfer = NRFX_SPI_XFER_TX(buf, 1);

    APP_ERROR_CHECK(nrfx_spi_xfer(hal_flash_spi, &xfer, 0));

    while (!m_spi_xfer_done) {
        __WFE();
    }
}