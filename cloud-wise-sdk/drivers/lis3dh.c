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
    uint8_t    temp_data = LIS3DH_WHO_AM_I_ADDR;

    err_code = nrfx_twim_tx(hal_lis3dh_twi, LIS3DH_ADDR, &temp_data, sizeof(temp_data), false);
    if (err_code != NRFX_SUCCESS) {
        NRFX_LOG_ERROR("%s %s", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
        return false;
    }

    err_code = nrfx_twim_rx(hal_lis3dh_twi, LIS3DH_ADDR, &temp_data, sizeof(temp_data));
    if (err_code != NRFX_SUCCESS) {
        NRFX_LOG_ERROR("%s %s", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
        return false;
    }

    NRFX_LOG_INFO("%s LIS3DH ID 0x%x", __func__, temp_data);

    return true;
}
