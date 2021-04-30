#include "lis3dh.h"

#include "hal/hal_drivers.h"

#define NRF_LOG_MODULE_NAME cloud_wise_sdk_drivers_lis3dh
#define NRF_LOG_LEVEL CLOUD_WISE_DEFAULT_LOG_LEVEL
#include "nrfx_log.h"
NRF_LOG_MODULE_REGISTER();

#define LIS3DH_ADDR 0x19

#define LIS3DH_WHO_AM_I_ADDR 0x0F


unsigned char Acc_Table[ ACC_TABLE_DRIVER_SIZE* 2] =
{
  0x20, 0x57,
  0x21, 0x00,
  0x22, 0x40,
  0x23, 0xB0,
  0x24, 0x00,
  0x25, 0x08,
  0x26, 0x00,
  0x2E, 0x00,
  0x30, 0x00,
  0x32, 0x00,
  0x33, 0x00,
  0x34, 0x00,
  0x36, 0x40,
  0x37, 0x64,
  0x38, 0x00,
  0x3E, 0x02,
  0x3F, 0x10,
};

unsigned char Acc_Sleep_Table[ ACC_TABLE_SLEEP_SIZE* 2] =
{
  0x20, 0x00,
};



uint8_t lis3dh_read_reg(uint8_t reg);

bool lis3dh_init(void)
{
  uint8_t value;

  value = lis3dh_read_reg(LIS3DH_WHO_AM_I_ADDR);
  NRFX_LOG_INFO("%s LIS3DH ID 0x%x", __func__, value);

  return true;
/*
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
    */
}

uint8_t lis3dh_read_reg(uint8_t reg)
{
    ret_code_t err_code;
    uint8_t    tx_temp_data = reg;
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

    //NRFX_LOG_INFO("%s LIS3DH ID 0x%x", __func__, rx_temp_data);

    return rx_temp_data;
}

void lis3dh_read_buffer( uint8_t* buffer, uint8_t size, uint8_t reg)
{
    ret_code_t err_code;
    uint8_t    tx_temp_data = reg;
    //uint8_t    rx_temp_data;

    const nrfx_twim_xfer_desc_t xfer_tx = NRFX_TWIM_XFER_DESC_TX(LIS3DH_ADDR, &tx_temp_data, sizeof(tx_temp_data));
    const nrfx_twim_xfer_desc_t xfer_rx = NRFX_TWIM_XFER_DESC_RX(LIS3DH_ADDR, buffer, size);

    err_code = nrfx_twim_xfer(hal_lis3dh_twi, &xfer_tx, NRFX_TWIM_FLAG_TX_NO_STOP);
    if (err_code != NRFX_SUCCESS) {
        NRFX_LOG_ERROR("%s %s", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
        return;
    }

    err_code = nrfx_twim_xfer(hal_lis3dh_twi, &xfer_rx, 0);
    if (err_code != NRFX_SUCCESS) {
        NRFX_LOG_ERROR("%s %s", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
        return;
    }

}

void lis3dh_write_reg(uint8_t reg, uint8_t value)
{
    ret_code_t err_code;
    uint8_t    tx_temp_data[2];
    //uint8_t    rx_temp_data;

    tx_temp_data[0] = reg;
    tx_temp_data[1] = value;

    const nrfx_twim_xfer_desc_t xfer_tx = NRFX_TWIM_XFER_DESC_TX(LIS3DH_ADDR, tx_temp_data, sizeof(tx_temp_data));

    err_code = nrfx_twim_xfer(hal_lis3dh_twi, &xfer_tx, 0);
    if (err_code != NRFX_SUCCESS) {
        NRFX_LOG_ERROR("%s %s", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
        return;
    }
}


unsigned char configure_acc( unsigned char* table, unsigned char table_size)
{
  unsigned char i,j,reg;

  for (i=0, j=0 ; i<table_size ;  i++ , j+=2 )
  {
    lis3dh_write_reg( table[j] , table[j+1] );
  }

  for (i=0, j=0 ; i<table_size ;  i++ , j+=2 )
  {
    reg = lis3dh_read_reg( table[j] );

    if ( reg != table[j+1] )
      return 0;
  }

  return 1;
}
 