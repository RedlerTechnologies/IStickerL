#include "commands.h"

#include "Configuration.h"
#include "FreeRTOS.h"
#include "ble/ble_services_manager.h"
#include "ble_file_transfer.h"
#include "drivers/buzzer.h"
#include "events.h"
#include "flash_data.h"
#include "hal/hal_boards.h"
#include "hal/hal_drivers.h"
#include "logic/serial_comm.h"
#include "monitor.h"
#include "recording.h"
#include "semphr.h"
#include "task.h"
#include "tracking_algorithm.h"

#include <stdlib.h>
#include <string.h>

#define MAX_COMMAND_SIZE 16
#define MAX_PARAM_SIZE 16

extern DriverBehaviourState driver_behaviour_state;
extern DeviceConfiguration  device_config;
extern xSemaphoreHandle     clock_semaphore;

extern Calendar            absolute_time;
extern BleReadingFileState ble_reading_file_state;

xSemaphoreHandle sleep_semaphore;
xSemaphoreHandle command_semaphore;

IStickerErrorBits error_bits;

void print_all_parameters(void);
void run_command(int8_t command_index, uint8_t *param, uint8_t *param_result, uint8_t is_set_command, uint8_t source);

ConfigParameter parameter_list[NUM_OF_PARAMETERS] = {
    {"BEEP", NULL, PARAM_COMMAND},
    {"CALIBRATE", NULL, PARAM_COMMAND},
    {"SLPD", NULL, PARAM_COMMAND},
    {"SW_VERSION", NULL, PARAM_COMMAND},
    {"SW_BUILD", NULL, PARAM_COMMAND},
    {"DEVID", (uint32_t *)device_config.DeviceID, PARAM_STRING},
    {"BLEID", (uint32_t *)device_config.DeviceName, PARAM_STRING},
    {"TIME", NULL, PARAM_COMMAND},
    {"RESET", NULL, PARAM_COMMAND},
    {"MEM_CLEAR", NULL, PARAM_COMMAND},
    {"RECORD", NULL, PARAM_COMMAND},
    {"FILE", NULL, PARAM_COMMAND},
    {"BLE", NULL, PARAM_COMMAND},
    {"TEST_MODE", NULL, PARAM_COMMAND},
    {"ACCIDENT_G", (uint32_t *)&device_config.AccidentG, PARAM_NUMERIC},
    {"SAVE", NULL, PARAM_COMMAND},
    {"MANUF", NULL, PARAM_COMMAND},
    {"SETTINGS", NULL, PARAM_COMMAND},
};

bool command_decoder(uint8_t *command_str, uint8_t max_size, uint8_t *result_buffer, uint8_t source)
{
    static uint8_t command[MAX_COMMAND_SIZE];
    static uint8_t param[MAX_PARAM_SIZE];

    static ConfigParameter *p = NULL;
    uint8_t                 i, j;
    int8_t                  index          = -1;
    uint8_t                 is_set_command = 1;
    bool                    flag           = true;

    xSemaphoreTake(command_semaphore, portMAX_DELAY);

    i = 0;
    while (i < max_size) {

        if (command_str[i] == 'P')
            break;

        i++;
    }

    if (command_str[i] == 'P') {
        memset(command, 0x00, MAX_COMMAND_SIZE);
        i += 2;
        j = 0;

        while (i < max_size) {
            command[j] = command_str[i];

            if (j < MAX_COMMAND_SIZE)
                j++;
            else {
                flag = false;
                break;
            }

            i++;

            if (command_str[i] <= 0x20)
                break;
        }

        if (strlen(command) > 0) {
            memset(param, 0x00, MAX_PARAM_SIZE);
            j = 0;
            i++;

            while (i < max_size) {
                param[j] = command_str[i];

                if (j < MAX_PARAM_SIZE)
                    j++;
                else {
                    flag = false;
                    break;
                }

                i++;

                if (command_str[i] <= 0x20)
                    break;
            }

            /*
            if (strlen(param) > 0) {

            } else {
                flag = false;
            }
            */

        } else
            flag = false;
    } else
        flag = false;

    vTaskDelay(10);

    if (param[0] == '?')
        is_set_command = 0;

    if (flag) {
        DisplayMessage("\r\nCOMMAND OK\r\n", 0, true);

        for (i = 0; i < NUM_OF_PARAMETERS; i++) {
            p = &parameter_list[i];
            if (strcmp(command, p->param_name) == 0) {
                index = i;
                break;
            }
        }

        run_command(index, param, result_buffer, is_set_command, source);
    } else {
        DisplayMessage("\r\nCOMMAND ERROR\r\n", 0, true);
    }

    terminal_buffer_lock();

    DisplayMessage("\r\nResponse: \r\n", 0, false);
    if (strlen(result_buffer) > 0)
        DisplayMessage(result_buffer, 0, false);
    DisplayMessage("\r\n", 0, false);

    terminal_buffer_release();

    xSemaphoreGive(command_semaphore);

    return flag;
}

void run_command(int8_t command_index, uint8_t *param, uint8_t *param_result, uint8_t is_set_command, uint8_t source)
{
    static ConfigParameter *p;
    int32_t                 param_num      = 0;
    int32_t                 result         = -1;
    uint8_t                 numeric_result = 1;
    bool                    is_remote      = (source == 1); // ble

    param_num = atoi(param);
    p         = &parameter_list[command_index];

    switch (command_index) {
    case COMMAND_BEEP:
        if (is_set_command) {
            buzzer_train(param_num);
            result = param_num;
        }
        break;

    case COMMAND_CALIBRATE:
        if (is_set_command) {
            driver_behaviour_state.calibrated        = false;
            driver_behaviour_state.store_calibration = true;
            result                                   = 0;
        }
        break;

    case COMMAND_SLEEP:
        set_sleep_timeout(param_num);
        driver_behaviour_state.manual_delayed = true;
        result                                = param_num;
        break;

    case COMMAND_SW_VERSION:
        if (!is_set_command)
            result = APP_MAJOR_VERSION * 256 + APP_MINOR_VERSION;
        break;

    case COMMAND_BUILD:
        if (!is_set_command)
            result = APP_BUILD;
        break;

    case COMMAND_DEVICE_ID:
    case COMMAND_BLE_ID:

        if (!is_set_command) {
            strcpy(param_result, (uint8_t *)p->param_address);
            numeric_result = 0;
        }

        break;

    case COMMAND_TIME:
        if (is_set_command) {
            // static Calendar calendar;

            xSemaphoreTake(clock_semaphore, portMAX_DELAY);

            SetTimeFromString(param, param + 7);
            result                             = GetTimeStampFromDate();
            driver_behaviour_state.time_synced = true;

            xSemaphoreGive(clock_semaphore);

            CreateGeneralEvent(result, EVENT_TYPE_TIME_SET, 4);
        }
        break;

    case COMMAND_RESET:
        if (is_set_command) {
            ActivateSoftwareReset(RESET_COMMAND, 0, 0, 0);
        }
        break;

    case COMMAND_CLEAR_MEMORY:
        if (is_set_command) {

            switch (param_num) {

            case 1:
                flash_erase_sectors_in_range(FLASH_RECORDS_START_ADDRESS, END_OF_FLASH);
                record_init();
                result = param_num;
                break;
            }
        }
        break;

    case COMMAND_RECORD:

        if (is_set_command) {
            if (param_num > 0) {
                param_num = record_search(param_num);
                param_num = -param_num;
            }

            if (param_num <= 0) {
                if (!is_remote)
                    record_print(-param_num);
            }

            if (is_remote) {
                // send record file here
                ble_reading_file_state.file_type = FILE_TYPE_RECORD;
                BFT_start(-param_num, 1);
                result = param_num;
            }
        } else {
            record_scan_for_new_records(true);
        }
        break;

    case COMMAND_FILE:

        if (is_set_command) {
            if (param_num == 0) {
                ble_reading_file_state.file_type             = param_num;
                driver_behaviour_state.new_transfer_protocol = true;
                BFT_start(param_num, 0);
            }
        }
        break;

    case COMMAND_BLE:
        if (is_set_command) {
            switch (param_num) {

            case 9:
                ble_services_disconnect();
                set_sleep_timeout(SLEEP_TIMEOUT_ON_ROUTE_BLE_DISCONNECTED);
                result = param_num;
                break;

            case 14:
                driver_behaviour_state.new_transfer_protocol = false;
                BFT_start(0, 0);
                result = param_num;
                break;

            case 15:
                switch (ble_reading_file_state.state) {
                case 1:
                    ble_reading_file_state.state = 2;
                    break;

                default:
                    ble_reading_file_state.state = 0xFF;
                    break;
                }

                result = param_num;

                break;

            case 16:

                switch (ble_reading_file_state.state) {
                case 1:
                    ble_reading_file_state.state = 3;
                    break;

                default:
                    ble_reading_file_state.state = 0xFF;
                    break;
                }

                result = param_num;

                break;
            }
        }

        break;

    case COMMAND_TEST_MODE:

        switch (param_num) {

        case 8:
            driver_behaviour_state.record_triggered = true;
            break;

        case PRINT_SIGNAL_MODE_ENERGY:
        case PRINT_SIGNAL_MODE_GX:
            if (driver_behaviour_state.print_signal_mode == param_num)
                driver_behaviour_state.print_signal_mode = 0;
            else
                driver_behaviour_state.print_signal_mode = param_num;
            break;
        }

        result = param_num;

        break;

    case COMMAND_ACCIDENT_G:

        if (is_set_command) {
            if (param_num >= 4) {
                memcpy((uint32_t *)p->param_address, &param_num, 1);
            }

        } else {
            memset(&result, 0x00, 4);
            memcpy(&result, (uint32_t *)p->param_address, 1);
        }

        break;

    case COMMAND_SAVE:

        SaveConfiguration(false);
        result = param_num;
        break;

    case COMMAND_DEFAULT_MANUFACTURE:

        SetManufactureDefault();
        SaveConfiguration(true);
        break;

    case COMMAND_SETTINGS:
        if (is_set_command) {
            print_all_parameters();
            result = param_num;
        }
        break;

    default:
        result = -9;
        break;
    }

    switch (numeric_result) {
    case 1:
        ltoa(result, param_result, 10);
        break;
    }
}

/*
void delay_sleep(int32_t delay_in_seconds)
{
    uint32_t current_delay;

    xSemaphoreTake(sleep_semaphore, portMAX_DELAY);

    if (delay_in_seconds) {
        current_delay = timeDiff(xTaskGetTickCount(), driver_behaviour_state.last_activity_time) / 1000;
        current_delay = driver_behaviour_state.sleep_delay_time - current_delay;

        if (delay_in_seconds > current_delay) {
            driver_behaviour_state.last_activity_time = xTaskGetTickCount();
            driver_behaviour_state.sleep_delay_time   = delay_in_seconds;
        }
    } else {
        driver_behaviour_state.sleep_delay_time = delay_in_seconds;
    }

    xSemaphoreGive(sleep_semaphore);
}
*/

void print_all_parameters(void)
{
    ConfigParameter *param;
    uint8_t          i;

    terminal_buffer_lock();

    for (i = 0; i < NUM_OF_PARAMETERS; i++) {
        param = &parameter_list[i];

        switch (param->param_type) {
        case PARAM_COMMAND:
            sprintf(alert_str, "%s\r\n", param->param_name);
            break;

        case PARAM_STRING:
            sprintf(alert_str, "%s: %s\r\n", param->param_name, param->param_address);
            break;

        case PARAM_NUMERIC:
            sprintf(alert_str, "%: %d\r\n", param->param_name, (int32_t)(*param->param_address));
            break;
        }

        DisplayMessage(alert_str, 0, false);
        vTaskDelay(10);
    }

    terminal_buffer_release();
}