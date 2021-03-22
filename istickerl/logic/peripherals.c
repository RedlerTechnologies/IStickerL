#include "peripherals.h"

#include "ble/ble_services_manager.h"
#include "drivers/lis3dh.h"
#include "hal/hal.h"
#include "nrf_log_ctrl.h"
#include "nrfx_saadc.h"

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

    hal_init(hal_evt_handler);

    lis3dh_init();

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

    default:
        NRFX_LOG_WARNING("%s Unknown HAL_EVENT %d", __func__, event);
    }
}

uint8_t peripherals_read_battery_level(void)
{
    // TODO Implement
    return 100;
}
