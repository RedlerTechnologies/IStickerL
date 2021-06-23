#include "commands.h"

#include "Configuration.h"
#include "FreeRTOS.h"
#include "ble/ble_services_manager.h"
#include "ble/ble_task.h"
#include "ble_file_transfer.h"
#include "decoder.h"
#include "drivers/buzzer.h"
#include "event_groups.h"
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
#include "transfer_task.h"

#include <stdlib.h>
#include <string.h>

#define MAX_COMMAND_SIZE 16
#define MAX_PARAM_SIZE 16

extern DriverBehaviourState driver_behaviour_state;
extern DeviceConfiguration  device_config;
extern xSemaphoreHandle     clock_semaphore;
extern EventGroupHandle_t   transfer_event;
extern EventGroupHandle_t   transfer_confirm_event;

extern Calendar            absolute_time;
extern BleReadingFileState ble_reading_file_state;

xSemaphoreHandle sleep_semaphore;
xSemaphoreHandle command_semaphore;

IStickerErrorBits error_bits;

void print_all_parameters(void);
void run_command(int8_t command_index, uint8_t *param, uint8_t *param_result, uint8_t is_set_command, uint8_t source);

ConfigParameter parameter_list[NUM_OF_PARAMETERS] = {
    {"BEEP", NULL, PARAM_COMMAND, 0},
    {"CALIBRATE", NULL, PARAM_COMMAND, 0},
    {"SLPD", NULL, PARAM_COMMAND, 0},
    {"SW_VERSION", NULL, PARAM_COMMAND, 0},
    {"SW_BUILD", NULL, PARAM_COMMAND, 0},
    {"DEVID", (uint32_t *)device_config.DeviceID, PARAM_STRING, 28},
    {"BLEID", (uint32_t *)device_config.DeviceName, PARAM_STRING, 16},
    {"TIME", NULL, PARAM_COMMAND, 0},
    {"RESET", NULL, PARAM_COMMAND, 0},
    {"MEM_CLEAR", NULL, PARAM_COMMAND, 0},
    {"RECORD", NULL, PARAM_COMMAND, 0},
    {"FILE", NULL, PARAM_COMMAND, 0},
    {"BLE", NULL, PARAM_COMMAND, 0},
    {"TEST_MODE", NULL, PARAM_COMMAND, 0},
    {"ACCIDENT_G", (uint32_t *)&device_config.AccidentG, PARAM_NUMERIC, 1},
    {"SAVE", NULL, PARAM_COMMAND, 0},
    {"MANUF", NULL, PARAM_COMMAND, 0},
    {"SETTINGS", NULL, PARAM_COMMAND},
    {"DATATX", NULL, PARAM_COMMAND, 0},
    {"DBGMODE", (uint32_t *)&device_config.buzzer_mode, PARAM_NUMERIC, 1},
    {"FILTER_Z", (uint32_t *)&device_config.filter_z, PARAM_NUMERIC, 2},
    {"OFFROAD_G", (uint32_t *)&device_config.offroad_g, PARAM_NUMERIC, 2},
    {"OFFROAD_PER", (uint32_t *)&device_config.offroad_per, PARAM_NUMERIC, 1},
    {"OFFROAD_DIS", (uint32_t *)&device_config.config_flags, PARAM_NUMERIC, 13},
    {"BUMPER_DIS", (uint32_t *)&device_config.config_flags, PARAM_NUMERIC, 1},
    {"TAMPER_MIN", (uint32_t *)&device_config.min_events_for_tamper, PARAM_NUMERIC, 1},
    {"TAMPER_ANGLE1", (uint32_t *)&device_config.tamper_angle1, PARAM_NUMERIC, 1},
    {"TAMPER_ANGLE2", (uint32_t *)&device_config.tamper_angle2, PARAM_NUMERIC, 1},

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

        if (command_str[i] == 'B')
            break;

        i++;
    }

    if (command_str[i] == 'P' || command_str[i] == 'B') {
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

            CreateDebugEvent(EVENT_DEBUG_SYNC_TIME, (SYNC_TIME_BY_COMMAND | SYNC_TIME_BEFORE), false);

            xSemaphoreTake(clock_semaphore, portMAX_DELAY);

            SetTimeFromString(param, param + 7);
            result                             = GetTimeStampFromDate();
            driver_behaviour_state.time_synced = true;

            xSemaphoreGive(clock_semaphore);

            CreateDebugEvent(EVENT_DEBUG_SYNC_TIME, (SYNC_TIME_BY_COMMAND | SYNC_TIME_AFTER), false);

            // CreateGeneralEvent(result, EVENT_TYPE_TIME_SET, 4);
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

            case 0:
                flash_erase_sectors_in_range(FLASH_COUNTER_START_ADDRESS, END_OF_FLASH);
                result = param_num;

                ActivateSoftwareReset(RESET_AFTER_FLASH_CLEAR, 0, 0, 0);
                break;

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
                set_last_ble_command_time();

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

            case 2:
                // request events

                set_last_ble_command_time();

                if (!driver_behaviour_state.in_event_transfer_process) {
                    xEventGroupSetBits(transfer_event, EVENT_BLE_CONNECTION);
                    result = param_num;
                }
                break;

            case 3:
                // confirm sent events
                xEventGroupSetBits(transfer_confirm_event, EVENT_BLE_TRANSFER_SUCCESS);
                result = param_num;
                break;

            case 4:
                // reject last sent events
                xEventGroupSetBits(transfer_confirm_event, EVENT_BLE_TRANSFER_ERROR);
                result = param_num;
                break;

            case 6:
                // abort last sent events
                xEventGroupSetBits(transfer_event, EVENT_BLE_TRANSFER_ABORT);
                result = param_num;
                break;

            case 11:
                xEventGroupSetBits(transfer_event, EVENT_BLE_BLOCK_CONFIRM_AND_EXIT);
                result = param_num;
                break;

            case 10:
                // keep connection active
                // enable this line only after the app stops sending unnecessary BLE 10 commands
                // ???????????? driver_behaviour_state.last_ble_command_time = xTaskGetTickCount();
                break;

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

        case TEST_MODE_TRIGGER_ACCIDENT:
            driver_behaviour_state.record_triggered = true;
            break;

        case PRINT_SIGNAL_MODE_ENERGY:
        case PRINT_SIGNAL_MODE_GX:
        case PRINT_SIGNAL_MODE_GY:
        case PRINT_SIGNAL_MODE_GZ:
            if (driver_behaviour_state.print_signal_mode == param_num)
                driver_behaviour_state.print_signal_mode = 0;
            else
                driver_behaviour_state.print_signal_mode = param_num;
            break;

        case TEST_MODE_PRINT_ALL_EVENTS:
            PrintAllEventData();
            break;

        case TEST_MODE_PRINT_ALL_EVENTS_AND_DATA:
            driver_behaviour_state.print_event_data = true;
            PrintAllEventData();
            driver_behaviour_state.print_event_data = false;

            break;

        case 3:
            driver_behaviour_state.registration_mode = true;
            break;

        case 4:
            driver_behaviour_state.registration_mode = false;
            break;

        case TEST_MODE_PRINT_SENT_EVENT_MODE:
            driver_behaviour_state.print_sent_event_id_mode = !driver_behaviour_state.print_sent_event_id_mode;
            break;

        case TEST_MODE_DROP_NEW_EVENTS:
            driver_behaviour_state.block_new_events = !driver_behaviour_state.block_new_events;
            break;

        case TEST_MODE_FILL_EVENT_FLASH:
            driver_behaviour_state.fill_event_flash = !driver_behaviour_state.fill_event_flash;
            break;

        case TEST_MODE_PRINT_SENT_DATA_MODE:
            driver_behaviour_state.print_sent_event_data_mode = !driver_behaviour_state.print_sent_event_data_mode;
            break;
        }

        result = param_num;

        break;

        /*
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
        */

        /*
            case COMMAND_BUZZER_MODE:

                if (is_set_command) {
                    memcpy((uint8_t *)p->param_address, &param_num, 1);
                } else {
                    memset(&result, 0x00, 4);
                    memcpy(&result, (uint8_t *)p->param_address, 1);
                }

                break;
        */
    case COMMAND_FILTER_Z:
    case COMMAND_ACCIDENT_G:
    case COMMAND_BUZZER_MODE:
    case COMMAND_OFFROAD_G:
    case COMMAND_OFFROAD_PER:
    case COMMAND_TAMPER_MIN:
    case COMMAND_TAMPER_ANGLE1:
    case COMMAND_TAMPER_ANGLE2:

        if (is_set_command) {
            memcpy((uint8_t *)p->param_address, &param_num, p->size);
        } else {
            memset(&result, 0x00, 4);
            memcpy(&result, (uint8_t *)p->param_address, p->size);
        }

        break;

    case COMMAND_OFFROAD_DIS:
    case COMMAND_BUMPER_DIS:
        if (is_set_command) {

            memcpy(&result, (uint8_t *)p->param_address, 4);

            if (param_num)
                result |= (1 << p->size);
            else
                result &= ~(1 << p->size);

            memcpy((uint8_t *)p->param_address, &result, 4);

            if (param_num)
                result = 1;
            else
                result = 0;
        } else {
            memset(&result, 0x00, 4);
            memcpy(&result, (uint8_t *)p->param_address, 4);
            result = result & (1 << p->size);
            if (result)
                result = 1;
        }
        break;

    case COMMAND_SAVE:

        SaveConfiguration(false);
        result = param_num;
        break;

    case COMMAND_DEFAULT_MANUFACTURE:

        if (is_set_command) {

            SetManufactureDefault();
            SaveConfiguration(true);

            ActivateSoftwareReset(RESET_SET_MANUFACTURE_DEFAULT, 0, 0, 0);
        }

        break;

    case COMMAND_SETTINGS:
        if (is_set_command) {
            print_all_parameters();
            result = param_num;
        }
        break;

    case COMMAND_DATATX:
        if (is_set_command) {
            ActivateSoftwareReset(RESET_AFTER_CHANGING_RX_PTR, param_num, 0, 0);
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