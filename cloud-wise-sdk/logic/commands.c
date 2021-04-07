#include "commands.h"

#include "../ble/ble_services_manager.h"
#include "../drivers/buzzer.h"
#include "../logic/serial_comm.h"
#include "Configuration.h"
#include "FreeRTOS.h"
#include "hal/hal_boards.h"
#include "hal/hal_drivers.h"
#include "monitor.h"
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

extern Calendar absolute_time;

xSemaphoreHandle sleep_semaphore;
xSemaphoreHandle command_semaphore;

IStickerErrorBits error_bits;

void run_command(int8_t command_index, uint8_t *param, uint8_t *param_result, uint8_t is_set_command);

ConfigParameter parameter_list[NUM_OF_PARAMETERS] = {
    {"BEEP", NULL, 85, 0, PARAM_TYPE_STRING, 0, 0},
    {"CALIBRATE", NULL, 85, 0, PARAM_TYPE_STRING, 0, 0},
    {"SLPD", NULL, 85, 0, PARAM_TYPE_STRING, 0, 0},
    {"SW_VERSION", NULL, 45, 2, PARAM_TYPE_INTEGER, 0, 1},
    {"SW_BUILD", NULL, 46, 2, PARAM_TYPE_INTEGER, 0, 1},
    {"DEVID", (uint32_t *)device_config.DeviceID, 46, 26, PARAM_TYPE_STRING, 0, 1},
    {"BLEID", (uint32_t *)device_config.DeviceName, 46, 16, PARAM_TYPE_STRING, 0, 1},
    {"TIME", NULL, 46, 2, PARAM_TYPE_STRING, 0, 1},
};

bool command_decoder(uint8_t *command_str, uint8_t max_size, uint8_t source)
{
    static uint8_t result_buffer[128];

    static uint8_t command[MAX_COMMAND_SIZE];
    static uint8_t param[MAX_PARAM_SIZE];

    static ConfigParameter *p = NULL;
    uint8_t                 i, j;
    int8_t                  index          = -1;
    uint8_t                 is_set_command = 1;
    bool                    flag           = true;

    xSemaphoreTake(command_semaphore, portMAX_DELAY);

    if (command_str[0] == 'P') {
        memset(command, 0x00, MAX_COMMAND_SIZE);
        i = 2;
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

            if (strlen(param) > 0) {

            } else {
                flag = false;
            }

        } else
            flag = false;
    } else
        flag = false;

    vTaskDelay(10);

    if (param[0] == '?')
        is_set_command = 0;

    if (flag) {
        DisplayMessage("COMMAND OK\r\n", 0);

        for (i = 0; i < NUM_OF_PARAMETERS; i++) {
            p = &parameter_list[i];
            if (strcmp(command, p->param_name) == 0) {
                index = i;
                break;
            }
        }

        if (index >= 0) {
            run_command(index, param, result_buffer, is_set_command);
        } else
            flag = false;
    } else {
        DisplayMessage("COMMAND ERROR\r\n", 0);
    }

    DisplayMessage("Response: \r\n", 0);
    DisplayMessage(result_buffer, 0);
    DisplayMessage("\r\n", 0);

    if (source == 1) {
        ble_services_notify_command(result_buffer, strlen(result_buffer));
    }

    xSemaphoreGive(command_semaphore);

    return flag;
}

void run_command(int8_t command_index, uint8_t *param, uint8_t *param_result, uint8_t is_set_command)
{
    static ConfigParameter *p;
    int32_t                 param_num      = 0;
    int32_t                 result         = -1;
    uint8_t                 numeric_result = 1;

    param_num = atoi(param);
    p         = &parameter_list[command_index];

    // empty result buffer
    param_result[0] = 0x0;

    switch (command_index) {
    case COMMAND_BEEP:
        if (is_set_command) {
            buzzer_train(param_num);
            result = param_num;
        }
        break;

    case COMMAND_CALIBRATE:
        if (is_set_command) {
            driver_behaviour_state.calibrated = false;
            result                            = 0;
        }
        break;

    case COMMAND_SLEEP:
        delay_sleep(param_num);
        result = param_num;
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

            SetTimeFromString(param, param + 7, &absolute_time);
            result         = GetTimeStampFromDate(&absolute_time);

            xSemaphoreGive(clock_semaphore);
        }
        break;
    }

    switch (numeric_result) {
    case 1:
        ltoa(result, param_result, 10);
        break;
    }
}

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