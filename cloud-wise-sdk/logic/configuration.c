#include "configuration.h"

#include "FreeRTOS.h"
#include "drivers/flash.h"
#include "hal/hal_boards.h"
#include "hal/hal_drivers.h"
#include "logic/events.h"
#include "logic/serial_comm.h"
#include "task.h"
#include "tracking_algorithm.h"

#define CONFIGURATION_ADDRESS 0x0000

#include <string.h>

DeviceConfiguration device_config;

extern DriverBehaviourState driver_behaviour_state;

void LoadConfiguration(void)
{
    uint16_t crc;

    terminal_buffer_lock();

    flash_read_buffer(alert_str, CONFIGURATION_ADDRESS, 256);
    memcpy(&device_config, alert_str + 4, 256);

    crc = CRC16_Calc((uint8_t *)&device_config, 254, 0);

    terminal_buffer_release();

    if (crc != device_config.crc) {
        SetManufactureDefault();
        SaveConfiguration(true);
    }

    // device_config.max_sleep_time = 600; // 120;
    // strcpy ( device_config.DeviceName , "S-KKKK0000001" );

    // driver_behaviour_state.sleep_delay_time = device_config.max_sleep_time;

    // device_config.AccidentG = MIN_G_FOR_ACCIDENT_EVENT;
}

void SaveConfiguration(bool force)
{
    uint16_t crc;

    crc = CRC16_Calc((uint8_t *)&device_config, 254, 0);

    if (force || crc != device_config.crc) {

        device_config.crc = crc;

        flash_erase_sector(CONFIGURATION_ADDRESS);

        terminal_buffer_lock();

        memcpy(alert_str + 4, &device_config, 256);

        flash_write_buffer(alert_str, CONFIGURATION_ADDRESS, 256);

        terminal_buffer_release();

        DisplayMessage("\r\nConfiguration saved\r\n", 0, true);
    }
}

void SetManufactureDefault(void)
{

    memset(&device_config, 0x00, 254);

    device_config.AccidentG   = MIN_G_FOR_ACCIDENT_EVENT;
    device_config.buzzer_mode = BUZZER_MODE_ON;

    memset(&device_config.calibrate_value, 0x00, sizeof(CalibratedValue));

    device_config.crc = CRC16_Calc((uint8_t *)&device_config, 254, 0);
}