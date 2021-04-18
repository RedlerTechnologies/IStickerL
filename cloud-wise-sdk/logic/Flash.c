#include "Flash.h"

#include "nrf_drv_spi.h"

static volatile bool spi_xfer_done;

uint8_t ExternalFlash_ReadStatus(nrf_drv_spi_t const *const p_instance);


uint8_t ExternalFlash_ReadStatus(nrf_drv_spi_t const *const p_instance)
{
    uint8_t sendBuf[2] = {READ_STATUS_REG, 0}; // 05 read register
    uint8_t irecvBuf[2], recv_len = 2;

    spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(p_instance, sendBuf, sizeof(sendBuf), irecvBuf, recv_len));

    while (!spi_xfer_done) {
        __WFE();
    }
    return irecvBuf[1];
}