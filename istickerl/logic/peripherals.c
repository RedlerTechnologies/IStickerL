#include "peripherals.h"

#include "ble/ble_services_manager.h"
#include "drivers/buzzer.h"
#include "drivers/lis3dh.h"
#include "hal/hal.h"
#include "hal/hal_boards.h"
#include "nrf_log_ctrl.h"
#include "serial_comm.h"
#include "flash.h"

#define NRF_LOG_MODULE_NAME logic_peripherals
#define NRF_LOG_LEVEL CLOUD_WISE_DEFAULT_LOG_LEVEL
#include "nrfx_log.h"
NRF_LOG_MODULE_REGISTER();

char device_serial_number[SERIAL_NUMBER_WIDTH];

static void hal_evt_handler(const hal_event_type_t event);

void peripherals_init(void)
{
    uint32_t serial = hal_read_device_serial_number(device_serial_number, SERIAL_NUMBER_WIDTH);
    NRFX_LOG_INFO("%s Device Serial %u [%s]\n", __func__, serial, device_serial_number);

    hal_init(hal_evt_handler, flash_spi_event_handler);

    lis3dh_init();    

    buzzer_init();

    serial_comm_init();

    hal_interrupts_set(false, true);

    NRF_LOG_FLUSH();
}

static void hal_evt_handler(const hal_event_type_t event)
{
    switch (event) {
    case HAL_EVENT_NOTHING:
        NRFX_LOG_INFO("%s HAL_EVENT_NOTHING", __func__);
        break;

    case HAL_EVENT_LIS3DH_INT1:
        NRFX_LOG_INFO("%s HAL_EVENT_LIS3DH_INT1", __func__);
        break;

    case HAL_EVENT_LIS3DH_INT2:
        NRFX_LOG_INFO("%s HAL_EVENT_LIS3DH_INT2", __func__);
        break;

    case HAL_EVENT_UART0_RX:
        serial_comm_process_rx();
        break;

    default:
        NRFX_LOG_WARNING("%s Unknown HAL_EVENT %d", __func__, event);
    }
}

void peripherals_toggle_leds(void)
{
    nrf_gpio_pin_toggle(HAL_LED_GREEN);
    nrf_gpio_pin_toggle(HAL_LED_RED);
}

uint8_t peripherals_read_temperature(void)
{
    uint32_t temperature;
    sd_temp_get(&temperature);

    return ROUNDED_DIV(temperature, 4);
}

inline uint16_t peripherals_read_vdd(void)
{
    uint32_t vdd_adc = hal_read_vdd_raw();

    // INFO https://devzone.nordicsemi.com/nordic/nordic-blog/b/blog/posts/measuring-lithium-battery-voltage-with-nrf52
    vdd_adc *= 100 * 6 * 6;            // Convert to mV, Gain 1/6, Ref 0.6V
    return ROUNDED_DIV(vdd_adc, 4095); // Div by 4095 (12 bits)
}

uint8_t peripherals_read_battery_level(void)
{
    //
    return battery_level_in_percent(peripherals_read_vdd());
}