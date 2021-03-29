#include "lis3dh.h"

#include "hal/hal_drivers.h"

#define NRF_LOG_MODULE_NAME cloud_wise_sdk_drivers_lis3dh
#define NRF_LOG_LEVEL CLOUD_WISE_DEFAULT_LOG_LEVEL
#include "nrfx_log.h"
NRF_LOG_MODULE_REGISTER();

#define LIS3DH_ADDR 0x19

#define LIS3DH_WHO_AM_I_ADDR 0x0F

bool lis3dh_init(void)
{
    ret_code_t err_code;
    uint8_t    tx_temp_data = LIS3DH_WHO_AM_I_ADDR;
    uint8_t    rx_temp_data;

    const nrfx_twim_xfer_desc_t xfer_tx = NRFX_TWIM_XFER_DESC_TX(LIS3DH_ADDR, &tx_temp_data, sizeof(tx_temp_data));
    const nrfx_twim_xfer_desc_t xfer_rx = NRFX_TWIM_XFER_DESC_RX(LIS3DH_ADDR, &rx_temp_data, sizeof(rx_temp_data));

    err_code = nrfx_twim_xfer(hal_lis3dh_twi, &xfer_tx, NRFX_TWIM_FLAG_TX_NO_STOP);
    if (err_code != NRFX_SUCCESS) {
        NRFX_LOG_ERROR("%s %s", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
        return false;
    }

    err_code = nrfx_twim_xfer(hal_lis3dh_twi, &xfer_rx, 0);
    if (err_code != NRFX_SUCCESS) {
        NRFX_LOG_ERROR("%s %s", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
        return false;
    }

    NRFX_LOG_INFO("%s LIS3DH ID 0x%x", __func__, rx_temp_data);

    return true;
}