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

    // strcpy ( device_config.DeviceName , "S-KKKK0000001" );

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
    uint8_t i;

    memset(&device_config, 0x00, 254);

    device_config.AccidentG             = MIN_G_FOR_ACCIDENT_EVENT;
    device_config.buzzer_mode           = BUZZER_MODE_ON;
    device_config.filter_z              = 40;
    device_config.offroad_g             = 25;
    device_config.offroad_per           = 50;
    device_config.min_events_for_tamper = 15;
    device_config.tamper_angle1         = 35;
    device_config.tamper_angle2         = 15;

    memset(&device_config.config_flags, 0x00, 4);
    device_config.config_flags.offroad_disabled = 1; // 0 - ??????????
    device_config.config_flags.bumper_dis       = 1; // 0 - ??????????

    memset(&device_config.calibrate_value, 0x00, sizeof(CalibratedValue));

    // driver behaviour table

    for (i = 0; i < NUM_DRIVER_EVENT_TYPES; i++) {
        device_config.dr_bh_durations[i][0] = 3;
        device_config.dr_bh_durations[i][1] = 3;
        device_config.dr_bh_durations[i][2] = 3;

        device_config.dr_bh_gvalues[i][0] = 15;
        device_config.dr_bh_gvalues[i][1] = 15;
        device_config.dr_bh_gvalues[i][2] = 15;
    }

    device_config.dr_bh_gvalues[DRIVER_BEHAVIOR_EVENT_BRAKES][2] = DR_BRAKES_G_TH_3;
    device_config.dr_bh_gvalues[DRIVER_BEHAVIOR_EVENT_BRAKES][1] = DR_BRAKES_G_TH_2;
    device_config.dr_bh_gvalues[DRIVER_BEHAVIOR_EVENT_BRAKES][0] = DR_BRAKES_G_TH_1;

    device_config.dr_bh_durations[DRIVER_BEHAVIOR_EVENT_BRAKES][2] = DR_BRAKES_DURATION_TH_3;
    device_config.dr_bh_durations[DRIVER_BEHAVIOR_EVENT_BRAKES][1] = DR_BRAKES_DURATION_TH_2;
    device_config.dr_bh_durations[DRIVER_BEHAVIOR_EVENT_BRAKES][0] = DR_BRAKES_DURATION_TH_1;

    device_config.dr_bh_gvalues[DRIVER_BEHAVIOR_EVENT_ACCEL][2] = DR_ACCEL_G_TH_3;
    device_config.dr_bh_gvalues[DRIVER_BEHAVIOR_EVENT_ACCEL][1] = DR_ACCEL_G_TH_2;
    device_config.dr_bh_gvalues[DRIVER_BEHAVIOR_EVENT_ACCEL][0] = DR_ACCEL_G_TH_1;

    device_config.dr_bh_durations[DRIVER_BEHAVIOR_EVENT_ACCEL][2] = DR_ACCEL_DURATION_TH_3;
    device_config.dr_bh_durations[DRIVER_BEHAVIOR_EVENT_ACCEL][1] = DR_ACCEL_DURATION_TH_2;
    device_config.dr_bh_durations[DRIVER_BEHAVIOR_EVENT_ACCEL][0] = DR_ACCEL_DURATION_TH_1;

    device_config.dr_bh_gvalues[DRIVER_BEHAVIOR_SHARP_TURN][2] = DR_SHARP_TURN_G_TH_3;
    device_config.dr_bh_gvalues[DRIVER_BEHAVIOR_SHARP_TURN][1] = DR_SHARP_TURN_G_TH_2;
    device_config.dr_bh_gvalues[DRIVER_BEHAVIOR_SHARP_TURN][0] = DR_SHARP_TURN_G_TH_1;

    device_config.dr_bh_durations[DRIVER_BEHAVIOR_SHARP_TURN][2] = DR_SHARP_TURN_DURATION_TH_3;
    device_config.dr_bh_durations[DRIVER_BEHAVIOR_SHARP_TURN][1] = DR_SHARP_TURN_DURATION_TH_2;
    device_config.dr_bh_durations[DRIVER_BEHAVIOR_SHARP_TURN][0] = DR_SHARP_TURN_DURATION_TH_1;

/*
    device_config.dr_bh_gvalues[DRIVER_BEHAVIOR_SLALUM][0]   = DR_SLALUM_GFORCE_TH;
    device_config.dr_bh_durations[DRIVER_BEHAVIOR_SLALUM][0] = DR_SLALUM_MAX_DUR_BETWEEN_TURNS;
    device_config.dr_bh_durations[DRIVER_BEHAVIOR_SLALUM][1] = DR_SLALUM_MIN_TURNS;
*/

    device_config.crc = CRC16_Calc((uint8_t *)&device_config, 254, 0);
}