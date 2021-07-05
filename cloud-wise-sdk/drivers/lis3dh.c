#include "lis3dh.h"

#include "hal/hal_drivers.h"
// TODO This logic module should be optional
//#include "logic/serial_comm.h"

#define NRF_LOG_MODULE_NAME cloud_wise_sdk_drivers_lis3dh
#define NRF_LOG_LEVEL CLOUD_WISE_DEFAULT_LOG_LEVEL
#include "nrfx_log.h"
NRF_LOG_MODULE_REGISTER();

#define LIS3DH_ADDR 0x19

#define TWI_MULTI_READ 0x80

#define LIS3DH_WHO_AM_I_ADDR 0x0F

#ifdef ACC_SAMPLE_FREQ_100HZ
#define ACC_REG_20H 0x57
#endif

#ifdef ACC_SAMPLE_FREQ_200HZ
#define ACC_REG_20H 0x67
//#define ACC_REG_20H 0x27
#endif

#ifdef ACC_SAMPLE_FREQ_400HZ
#define ACC_REG_20H 0x77
#endif

typedef enum {
    LISDH_CTRL_REG1 = 0x20,
    LISDH_CTRL_REG2 = 0x21,
    LISDH_CTRL_REG3 = 0x22,
    LISDH_CTRL_REG4 = 0x23,
    LISDH_CTRL_REG5 = 0x24,
    LISDH_CTRL_REG6 = 0x25,

    LISDH_REFERENCE  = 0x26,
    LISDH_STATUS_REG = 0x27,

    LISDH_OUT_XL = 0x28,
    LISDH_OUT_XH = 0x29,
    LISDH_OUT_YL = 0x2A,
    LISDH_OUT_YH = 0x2B,
    LISDH_OUT_ZL = 0x2C,
    LISDH_OUT_ZH = 0x2D,

    LISDH_FIFO_CTRL_REG = 0x2E,
    LISDH_FIFO_SRC_REG  = 0x2F,

    LISDH_INT1_CFG = 0x30,
    LISDH_INT1_SRC = 0x31,
    LISDH_INT1_THS = 0x32,
    LISDH_INT1_DUR = 0x33,

    LISDH_INT2_CFG = 0x34,
    LISDH_INT2_SRC = 0x35,
    LISDH_INT2_THS = 0x36,
    LISDH_INT2_DUR = 0x37,

    LISDH_CLICK_CFG = 0x38,
    LISDH_CLICK_SRC = 0x39,
    LISDH_CLICK_THS = 0x3A,

    LISDH_TIME_LIMIT   = 0x3B,
    LISDH_TIME_LATENCY = 0x3C,
    LISDH_TIME_WINDOW  = 0x3D,

    LISDH_ACT_THS = 0x3E,
    LISDH_ACT_DUR = 0x3F,
} eLis3dh;

static uint8_t m_acc_init[] = {

    // low power mode disabled, sample rate 200hz, x,y,z axis enabled
    LISDH_CTRL_REG1,
    ACC_REG_20H,

    // normal mode, click filter bypassed, high pass filter disabled
    LISDH_CTRL_REG2,
    0x00,

    // click int disabled, ZYD data ready int disabled, FIFO watermark disabled,
    // FIFO overrun int disabled
    LISDH_CTRL_REG3,
    0x00,

    // BDU bit enabled, Full scale = 16g,
    // High resolution disabled, SPI 3-wire disabled, seld-test disabled
    LISDH_CTRL_REG4,
    0xB0,

    // FIFO enabled, BOOT disabled, 4D ditection int 1,2 disabled
    // latch int 1,2 disabled,
    LISDH_CTRL_REG5,
    0x40,

    // int 2 click disabled,
    //  activity interrupt disable
    LISDH_CTRL_REG6,
    0x00,

    // interrupt reference value
    LISDH_REFERENCE,
    0x00,

    // no fifo
    LISDH_FIFO_CTRL_REG,
    0x00,

    // no interrupt functionality
    LISDH_INT1_CFG,
    0x00,

    // int1 threshold 0
    LISDH_INT1_THS,
    0x00,

    // int1 duration 0
    LISDH_INT1_DUR,
    0x00,

    // x,y,z disable x,y,z axis
    LISDH_INT2_CFG,
    0x00,

    // int2 threhold
    LISDH_INT2_THS,
    0x00,

    // int 2 duration value = 100
    LISDH_INT2_DUR,
    0x00,

    // click interrupt disbaled
    LISDH_CLICK_CFG,
    0x00,

    // activity interrupt disabled
    LISDH_ACT_THS,
    0x00,

    // activity interrupt disabled
    LISDH_ACT_DUR,
    0x00,
};

static volatile bool m_xfer_done    = false;
static volatile bool m_read_buffers = false;

static bool    configure(uint8_t *table, uint8_t table_size);
static void    write_reg_blocking(uint8_t reg, uint8_t value);
static uint8_t read_reg_blocking(uint8_t reg);

#define BUFFER_LENGTH (32 * sizeof(uint16_t) * 3)
static uint8_t m_samples_buffer[BUFFER_LENGTH];

bool lis3dh_init(void)
{
    uint8_t value;

    value = read_reg_blocking(LIS3DH_WHO_AM_I_ADDR);
    NRFX_LOG_INFO("%s LIS3DH ID 0x%x", __func__, value);

    return configure(m_acc_init, sizeof(m_acc_init) >> 1);
}

void lis3dh_evt_handler(nrfx_twim_evt_t const *p_event, void *p_context)
{
    switch (p_event->type) {
    case NRFX_TWIM_EVT_DONE:
        m_xfer_done = true;

        if (m_read_buffers) {
            m_read_buffers = false;

            // NRFX_LOG_HEXDUMP_INFO(m_samples_buffer, p_event->xfer_desc.secondary_length);
        }
        break;

    default:
        NRFX_LOG_ERROR("%s %u", __func__, p_event->type);
        break;
    }
}

static uint8_t read_reg_blocking(uint8_t reg)
{
    ret_code_t     err_code;
    static uint8_t tx_temp_data;
    static uint8_t rx_temp_data;

    tx_temp_data = reg;

    const nrfx_twim_xfer_desc_t xfer_txrx =
        NRFX_TWIM_XFER_DESC_TXRX(LIS3DH_ADDR, &tx_temp_data, sizeof(tx_temp_data), &rx_temp_data, sizeof(rx_temp_data));

    m_xfer_done = false;

    err_code = nrfx_twim_xfer(hal_lis3dh_twi, &xfer_txrx, 0);
    if (err_code != NRFX_SUCCESS) {
        NRFX_LOG_ERROR("%s %s", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
        // TODO Fix
        return false;
    }

    while (m_xfer_done == false)
        ;

    // NRFX_LOG_INFO("%s LIS3DH ID 0x%x", __func__, rx_temp_data);

    return rx_temp_data;
}

void lis3dh_read_buffer(uint8_t *buffer, uint8_t size, uint8_t reg)
{
    ret_code_t     err_code;
    static uint8_t tx_temp_data;

    tx_temp_data = reg;

    const nrfx_twim_xfer_desc_t xfer_txrx = NRFX_TWIM_XFER_DESC_TXRX(LIS3DH_ADDR, &tx_temp_data, sizeof(tx_temp_data), buffer, size);

    m_xfer_done    = false;
    m_read_buffers = true;

    err_code = nrfx_twim_xfer(hal_lis3dh_twi, &xfer_txrx, 0);
    if (err_code != NRFX_SUCCESS) {
        NRFX_LOG_ERROR("%s %s", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
        return;
    }
}

static void write_reg_blocking(uint8_t reg, uint8_t value)
{
    ret_code_t err_code;
    uint8_t    tx_temp_data[2];

    tx_temp_data[0] = reg;
    tx_temp_data[1] = value;

    const nrfx_twim_xfer_desc_t xfer_tx = NRFX_TWIM_XFER_DESC_TX(LIS3DH_ADDR, tx_temp_data, sizeof(tx_temp_data));

    m_xfer_done = false;

    err_code = nrfx_twim_xfer(hal_lis3dh_twi, &xfer_tx, 0);
    if (err_code != NRFX_SUCCESS) {
        NRFX_LOG_ERROR("%s %s", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
        return;
    }

    while (m_xfer_done == false)
        ;
}

static bool configure(uint8_t *table, uint8_t table_size)
{
    unsigned char i, j, reg;
    bool          success = true;

    for (i = 0, j = 0; i < table_size; i++, j += 2) {
        write_reg_blocking(table[j], table[j + 1]);
    }

    for (i = 0, j = 0; i < table_size; i++, j += 2) {
        reg = read_reg_blocking(table[j]);

        if (reg != table[j + 1]) {
            success = false;
            NRFX_LOG_ERROR("%s 0x%02x 0x%02x 0x%02x", __func__, table[j], reg, table[j + 1]);
        }
    }

    return success;
}

bool lis3dh_configure_idle(void)
{
    uint8_t config[] = {
        // no fifo
        LISDH_FIFO_CTRL_REG,
        0x00,

        // activity interrupt disabled
        LISDH_ACT_THS,
        0x00,

        // activity interrupt disabled
        LISDH_ACT_DUR,
        0x00,

        // int 2 click disabled,
        //  activity interrupt disable
        LISDH_CTRL_REG6,
        0x00,
    };

    if (configure(config, sizeof(config) >> 1)) {
        NRFX_LOG_INFO("%s OK", __func__);
    } else {
        NRFX_LOG_ERROR("%s Failed", __func__);
    }
}

bool lis3dh_configure_sleep(void)
{
    uint8_t config[] = {
        // no fifo
        LISDH_FIFO_CTRL_REG,
        0x00,

        // activity interrupt enabled
        LISDH_ACT_THS,
        0x02,

        LISDH_ACT_DUR,
        0x10,

        // TODO CTRL_REG1 - move to low power (LPen)
        // LISDH_CTRL_REG1,
        // 0x3F,  //0x08
        // this definition cause problem to wakeup by movement.
        // need to recheck again...

        // int 2 click disabled,
        // activity interrupt enables (wakeup from deep sleep)
        LISDH_CTRL_REG6,
        0x08,
    };

    if (configure(config, sizeof(config) >> 1)) {
        NRFX_LOG_INFO("%s OK", __func__);
    } else {
        NRFX_LOG_ERROR("%s Failed", __func__);
    }
}

bool lis3dh_configure_fifo(void)
{
    uint8_t config[] = {
        // activity interrupt disabled
        LISDH_ACT_THS,
        0x00,

        // activity interrupt disabled
        LISDH_ACT_DUR,
        0x00,

        // int 2 click disabled,
        // activity interrupt disable
        LISDH_CTRL_REG6,
        0x00,

        // click int disabled, ZYD data ready int disabled, FIFO watermark enabled,
        // FIFO overrun int enabled
        LISDH_CTRL_REG3,
        0x04,

        // Stream-to-FIFO, INT1, FTH set to 15 (half as read to N+1)
        LISDH_FIFO_CTRL_REG,
        0xAF,

    };

    if (configure(config, sizeof(config) >> 1)) {
        NRFX_LOG_INFO("%s OK", __func__);
    } else {
        NRFX_LOG_ERROR("%s Failed", __func__);
    }
}

inline bool lis3dh_int_handler(void)
{
    // uint8_t int_src = read_reg_blocking(LISDH_FIFO_SRC_REG);
    // NRFX_LOG_INFO("%s %x", __func__, int_src);

    lis3dh_read_buffer(m_samples_buffer, BUFFER_LENGTH, LISDH_OUT_XL | TWI_MULTI_READ);
}
