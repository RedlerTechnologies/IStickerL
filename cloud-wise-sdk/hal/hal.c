#include "hal.h"

#include "drivers/flash.h"
#include "hal_boards.h"
#include "hal_drivers.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log_ctrl.h"
#include "nrfx_gpiote.h"
#include "nrfx_saadc.h"

#include <string.h>

#define NRF_LOG_MODULE_NAME cloud_wise_sdk_hal
#define NRF_LOG_LEVEL CLOUD_WISE_DEFAULT_LOG_LEVEL
#include "nrfx_log.h"
NRF_LOG_MODULE_REGISTER();

static const nrfx_uart_t m_uart0 = NRFX_UART_INSTANCE(0);
static const nrfx_twim_t m_twim0 = NRFX_TWIM_INSTANCE(0);
static const nrfx_pwm_t  m_pwm0  = NRFX_PWM_INSTANCE(0);
static const nrfx_spi_t  m_spi1  = NRFX_SPI_INSTANCE(1);

const nrfx_twim_t *hal_lis3dh_twi = &m_twim0;
const nrfx_uart_t *hal_uart       = &m_uart0;
const nrfx_pwm_t * hal_buzzer     = &m_pwm0;
const nrfx_spi_t * hal_flash_spi  = &m_spi1;

volatile struct {
    bool lis3dh_int1;
    bool lis3dh_int2;
} m_int_enable = {0};

static void init_gpio(void);
static void init_uart(void);
static void init_twim(nrfx_twim_evt_handler_t handler);
static void init_spi(nrfx_spi_evt_handler_t handler);
static void init_saadc(void);
static void init_saadc_channels(void);
static void calibrate_saadc(void);
static void init_pwm(void);

static void uart0_event_handler(nrfx_uart_event_t const *p_event, void *p_context);
static void saadc_event_handler(nrfx_saadc_evt_t const *p_event);

static hal_evt_handler_t p_evt_handler;

void hal_init(hal_evt_handler_t evt_handler, nrfx_spi_evt_handler_t spi_handler, nrfx_twim_evt_handler_t twim_handler)
{
    ret_code_t ret;

    ASSERT(evt_handler);

    p_evt_handler = evt_handler;

    init_gpio();

    // blinks led to indicate a reset for the user
    nrf_gpio_pin_clear(HAL_LED_GREEN);
    nrf_gpio_pin_clear(HAL_LED_RED);
    nrf_delay_ms(100);
    nrf_gpio_pin_set(HAL_LED_GREEN);
    nrf_gpio_pin_set(HAL_LED_RED);

    init_twim(twim_handler);
    init_spi(spi_handler);
    init_pwm();
    init_uart();
    init_saadc();
}

static void gpiote_event_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    switch (pin) {
    case HAL_LIS3DH_INT1:
        if (m_int_enable.lis3dh_int1)
            p_evt_handler(HAL_EVENT_LIS3DH_INT1);
        break;

    case HAL_LIS3DH_INT2:
        if (m_int_enable.lis3dh_int2)
            p_evt_handler(HAL_EVENT_LIS3DH_INT2);
        break;

    default:
        NRFX_LOG_WARNING("%s Unknown INT Pin %u", __func__, pin);
    }
}

static void init_gpio(void)
{
    ret_code_t err_code;
    if (nrfx_gpiote_is_init() == false) {
        err_code = nrfx_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }

    nrf_gpio_cfg_output(HAL_LED_GREEN);
    nrf_gpio_cfg_output(HAL_LED_RED);

    nrf_gpio_cfg_output(HAL_SPI_FLASH_RESETN);
    nrf_gpio_pin_set(HAL_SPI_FLASH_RESETN);


    nrfx_gpiote_in_config_t in_config;

    in_config      = (nrfx_gpiote_in_config_t)NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    in_config.pull = NRF_GPIO_PIN_NOPULL;
    err_code       = nrfx_gpiote_in_init(HAL_LIS3DH_INT1, &in_config, gpiote_event_handler);
    if (NRFX_SUCCESS != err_code)
        NRFX_LOG_ERROR("%s nrfx_gpiote_in_init failed: %s", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
    APP_ERROR_CHECK(err_code);

    in_config      = (nrfx_gpiote_in_config_t)NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    in_config.pull = NRF_GPIO_PIN_NOPULL;
    err_code       = nrfx_gpiote_in_init(HAL_LIS3DH_INT2, &in_config, gpiote_event_handler);
    if (NRFX_SUCCESS != err_code)
        NRFX_LOG_ERROR("%s nrfx_gpiote_in_init failed: %s", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
    APP_ERROR_CHECK(err_code);
}

static void init_uart(void)
{
    ret_code_t err_code;

    nrfx_uart_config_t uart_config = NRFX_UART_DEFAULT_CONFIG;

    uart_config.pseltxd = HAL_USART0_TX;
    uart_config.pselrxd = HAL_USART0_RX;

    err_code = nrfx_uart_init(&m_uart0, &uart_config, uart0_event_handler);
    if (NRFX_SUCCESS != err_code)
        NRFX_LOG_ERROR("%s nrfx_uart_init failed: %s", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
    APP_ERROR_CHECK(err_code);
}

static void init_twim(nrfx_twim_evt_handler_t handler)
{
    nrfx_err_t err_code;

    const nrfx_twim_config_t twi_config = {.scl                = HAL_TWIM0_SCL,
                                           .sda                = HAL_TWIM0_SDA,
                                           .frequency          = HAL_TWIM0_FREQ,
                                           .interrupt_priority = NRFX_TWIM_DEFAULT_CONFIG_IRQ_PRIORITY,
                                           .hold_bus_uninit    = false};

    err_code = nrfx_twim_init(&m_twim0, &twi_config, handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrfx_twim_enable(&m_twim0);
}

static void init_spi(nrfx_spi_evt_handler_t handler)
{
    nrfx_err_t err_code;

    nrfx_spi_config_t spi_config = NRFX_SPI_DEFAULT_CONFIG;

    // spi_config.frequency = NRF_SPI_FREQ_4M; (default rate)
    spi_config.frequency = NRF_SPI_FREQ_8M;

    spi_config.sck_pin  = HAL_SPI1_CLK;
    spi_config.ss_pin   = HAL_SPI1_SS;
    spi_config.mosi_pin = HAL_SPI1_MOSI;
    spi_config.miso_pin = HAL_SPI1_MISO;
    spi_config.mode     = NRF_SPI_MODE_0;

    err_code = nrfx_spi_init(&m_spi1, &spi_config, handler, NULL);
    if (NRFX_SUCCESS != err_code)
        NRFX_LOG_ERROR("%s nrfx_spi_init failed: %s", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
    APP_ERROR_CHECK(err_code);
}

static void init_saadc(void)
{
    nrfx_err_t err_code;

    err_code = nrfx_saadc_init(NRFX_SAADC_CONFIG_IRQ_PRIORITY);
    APP_ERROR_CHECK(err_code);

    calibrate_saadc();
    init_saadc_channels();
}

static void calibrate_saadc(void)
{
    nrfx_err_t err_code;

    err_code = nrfx_saadc_offset_calibrate(NULL);
    APP_ERROR_CHECK(err_code);
}

static void init_saadc_channels(void)
{
    nrfx_err_t err_code;

    nrfx_saadc_channel_t channels[] = {NRFX_SAADC_DEFAULT_CHANNEL_SE(HAL_ADC_VBATT, HAL_SAADC_VBAT_CHANNEL)};

    channels[0].channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL;
    channels[0].channel_config.gain      = NRF_SAADC_GAIN1_6;
    channels[0].channel_config.acq_time  = NRF_SAADC_ACQTIME_40US;

    err_code = nrfx_saadc_channels_config(channels, sizeof(channels) / sizeof(nrfx_saadc_channel_t));
    if (NRFX_SUCCESS != err_code)
        NRFX_LOG_ERROR("%s nrfx_saadc_channels_config failed: %s", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
    APP_ERROR_CHECK(err_code);
}

static void init_pwm(void)
{
    nrfx_err_t err_code = NRFX_SUCCESS;

    const nrfx_pwm_config_t pwm_config = {
        .output_pins  = {HAL_BUZZER_PWM, NRFX_PWM_PIN_NOT_USED, NRFX_PWM_PIN_NOT_USED, NRFX_PWM_PIN_NOT_USED},
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .base_clock   = NRF_PWM_CLK_2MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = HAL_BUZZER_PWM_CYCLE_COUNT,
        .load_mode    = NRF_PWM_LOAD_COMMON,
        .step_mode    = NRF_PWM_STEP_AUTO};

    err_code = nrfx_pwm_init(&m_pwm0, &pwm_config, NULL);
    APP_ERROR_CHECK(err_code);
}

uint32_t hal_read_device_serial_number(char *const p_serial, uint8_t max_len)
{
    uint32_t serial_number = NRF_UICR->CUSTOMER[HAL_UICR_DEVICE_SERIAL_NUMBER];

    if (serial_number == UINT32_MAX || serial_number == 0x0) {
        if (p_serial)
            snprintf(p_serial, max_len, "N/A");

        return 0;
    }

    if (p_serial) {
        snprintf(p_serial, max_len, "%0*X", max_len - 1, serial_number);
    }

    NRFX_LOG_INFO("%s Serial 0x%x (%u)", __func__, serial_number, serial_number);

    return serial_number;
}

void hal_interrupts_set(bool enable_int1, bool enable_int2)
{
    m_int_enable.lis3dh_int1 = enable_int1;
    m_int_enable.lis3dh_int2 = enable_int2;
    nrfx_gpiote_in_event_enable(HAL_LIS3DH_INT1, enable_int1);
    nrfx_gpiote_in_event_enable(HAL_LIS3DH_INT2, enable_int2);
}

static void uart0_event_handler(nrfx_uart_event_t const *p_event, void *p_context)
{
    switch (p_event->type) {
    case NRFX_UART_EVT_RX_DONE:
        p_evt_handler(HAL_EVENT_UART0_RX);
        break;

    case NRFX_UART_EVT_TX_DONE:
        // p_evt_handler(HAL_EVENT_UART0_TX);
        NRFX_LOG_DEBUG("%s TX %u", __func__, p_event->data.rxtx.bytes);
        break;

    case NRFX_UART_EVT_ERROR:
        NRFX_LOG_ERROR("%s Error", __func__);
        break;
    }
}

static void saadc_event_handler(nrfx_saadc_evt_t const *p_event)
{
    //
}

int16_t hal_read_vdd_raw(void)
{
    nrfx_err_t        err_code;
    nrf_saadc_value_t battery_voltage[1];
    uint32_t          channels = (1 << HAL_SAADC_VBAT_CHANNEL);

    err_code = nrfx_saadc_simple_mode_set(channels, NRFX_SAADC_CONFIG_RESOLUTION, NRFX_SAADC_CONFIG_OVERSAMPLE, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_saadc_buffer_set(battery_voltage, 1);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_saadc_mode_trigger();
    APP_ERROR_CHECK(err_code);

    if (err_code != NRFX_SUCCESS) {
        NRFX_LOG_ERROR("%s %s", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
        return UINT8_MAX;
    }

    return battery_voltage[0];
}

uint8_t hal_scan_twim0(void)
{
    uint8_t    detected_devices = 0;
    uint8_t    address;
    ret_code_t err_code;
    uint8_t    temp_data;

    NRFX_LOG_INFO("%s TWIM0 scanner started", __func__);
    for (address = 1; address <= 127; ++address) {
        err_code = nrfx_twim_rx(&m_twim0, address, &temp_data, sizeof(temp_data));

        if (err_code == NRFX_SUCCESS) {
            ++detected_devices;
            NRFX_LOG_INFO("%s TWIM0 device detected at address 0x%x (%d).", __func__, address, address);
        }
    }

    if (detected_devices == 0) {
        NRFX_LOG_WARNING("%s no device was found on TWIM0", __func__);
    } else {
        NRFX_LOG_INFO("%s %d devices were found on TWIM0", __func__, detected_devices);
    }

    return detected_devices;
}

void isticker_bsp_board_sleep(void)
{
    nrf_gpio_pin_set(HAL_LED_RED);
    nrf_gpio_pin_set(HAL_LED_GREEN);
}
