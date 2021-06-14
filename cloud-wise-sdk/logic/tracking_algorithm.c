#include "tracking_algorithm.h"

#include "FreeRTOS.h"
#include "ble/ble_services_manager.h"
#include "ble/ble_task.h"
#include "ble_file_transfer.h"
#include "commands.h"
#include "configuration.h"
#include "decoder.h"
#include "drivers/buzzer.h"
#include "drivers/lis3dh.h"
#include "event_groups.h"
#include "events.h"
#include "flash_data.h"
#include "float.h"
#include "hal/hal.h"
#include "hal/hal_boards.h"
#include "hal/hal_drivers.h"
#include "logic/clock.h"
#include "logic/serial_comm.h"
#include "monitor.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_power.h"
#include "nrf_pwr_mgmt.h"
#include "nrfx_gpiote.h"
#include "nrfx_log.h"
#include "recording.h"
#include "task.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern uint8_t             Acc_Table[];
extern uint8_t             Acc_Sleep_Table[];
extern DeviceConfiguration device_config;
extern uint32_t            reset_count_x;
extern ResetData           reset_data;
extern DeviceConfiguration device_config;
extern ScanResult          scan_result;
extern EventGroupHandle_t  event_acc_process_sample;

extern AccSample acc_samples1[SAMPLE_BUFFER_SIZE];

AccConvertedSample *incoming_sample_ptr;

ResetData reset_data_copy;

DriverBehaviourState driver_behaviour_state;

void           calculate_sample(AccSample *acc_sample, uint8_t *buffer);
unsigned short GetGValue(AccConvertedSample *sample);
void           ProcessDrivingState(void);
void           ACC_CalibrateSample(AccSample *acc_sample_in, AccConvertedSample *acc_sample_out);
void           Process_Accident(DriverBehaviourState *state, AccConvertedSample *sample);
void           Process_Calibrate(void);
void           InitWakeupAlgorithm(void);
signed short   calculate_accident_hit_angle(AccConvertedSample *sample);
bool           ProcessWakeupState(void);
bool           CheckNoActivity(void);
void           Calculate_Energy(void);
static void    terminal_print_signal_mode(void);
void           send_calibration_alert(void);
void           print_movment(void);
void           Set_Installation_Angle(void);
uint16_t       get_current_state_timeout(void);
void           send_acc_sample_to_ble(void);
uint16_t       GetMinEneryForMovement(void);
void           ProcessTamper(void);
void           TestTamperState(void);

extern AccConvertedSample *incoming_sample_ptr;

void driver_behaviour_task(void *pvParameter)
{
    static unsigned  duration;
    static AccSample acc_sample;
    static uint32_t  search_id = 0;
    uint8_t          count     = 0;
    EventBits_t      uxBits;
    bool             need_sleep;
    bool             sleep_flag;

    buzzer_train(1);

    if ((RECORD_SIZE - RECORD_HEADER_SIZE - RECORD_TERMINATOR_SIZE) < SAMPLE_MEMORY_SIZE) {
        DisplayMessage("error in memory size", 0, true);
        ActivateSoftwareReset(RESET_OUT_OF_MEMORY, 0, 0, 0);
    }

    LoadConfiguration();
    ble_services_init_0();
    SaveConfiguration(false);
    ble_services_init();

#ifdef BLE_ADVERTISING
    ble_services_advertising_start();
#endif

    configure_acc(Acc_Table, ACC_TABLE_DRIVER_SIZE);

    // init glabal state variables //

    driver_behaviour_state.time_synced                = false;
    driver_behaviour_state.record_triggered           = false;
    driver_behaviour_state.track_state                = TRACKING_STATE_WAKEUP;
    driver_behaviour_state.stop_advertising_time      = 0;
    driver_behaviour_state.store_calibration          = false;
    driver_behaviour_state.acc_int_counter            = 0;
    driver_behaviour_state.manual_delayed             = false;
    driver_behaviour_state.energy                     = -1;
    driver_behaviour_state.print_signal_mode          = 0;
    driver_behaviour_state.print_sent_event_id_mode   = false;
    driver_behaviour_state.in_event_transfer_process  = false;
    driver_behaviour_state.block_new_events           = false;
    driver_behaviour_state.fill_event_flash           = false;
    driver_behaviour_state.print_sent_event_data_mode = false;
    driver_behaviour_state.registration_mode          = false;

    // calibration value
    if (device_config.calibrate_value.avg_value.Z == 0) {
        driver_behaviour_state.calibrated = false;
    } else {
        driver_behaviour_state.calibrated = true;
        memcpy(&driver_behaviour_state.calibrated_value, &device_config.calibrate_value, sizeof(CalibratedValue));
        Set_Installation_Angle();
    }

    driver_behaviour_state.calibratation_saved_in_flash = driver_behaviour_state.calibrated;

    driver_behaviour_state.tampered = !driver_behaviour_state.calibrated;

    InitWakeupAlgorithm();

    driver_behaviour_state.sleep_delay_time = 40;

    if (reset_data.reason == RESET_AFTER_CHANGING_RX_PTR) {
        search_id = reset_data.v1;
    }

    terminal_buffer_lock();
    sprintf(alert_str, "\r\n\r\nISticker-L version: %d.%d.%d - reset=%d reason=%d,%x,%x,%x\r\n\r\n", APP_MAJOR_VERSION, APP_MINOR_VERSION,
            APP_BUILD, reset_count_x, reset_data.reason, reset_data.v1, reset_data.v2, reset_data.v3);
    DisplayMessage(alert_str, 0, false);
    terminal_buffer_release();

    memcpy(&reset_data_copy, &reset_data, sizeof(ResetData));
    memset(&reset_data, 0x00, sizeof(ResetData));

    /*
    // test external flash
    uint16_t flash_id = flash_read_manufacture_id();
    NRFX_LOG_INFO("%s Flash ID 0x%04X", __func__, flash_id);
    */

    record_init();
    set_sleep_timeout_on_ble();

    // init counter
    flash_counter_read(FLASH_COUNTER_IDX_LAST_SENT_EVENT_ID, (uint8_t *)&scan_result.read_marker.event_id, 4);
    flash_counter_read(FLASH_COUNTER_IDX_LAST_SENT_EVENT_ADDRESS, (uint8_t *)&scan_result.read_marker.event_id, 4);

    InitEventFlashStructure(search_id);

    CreateVersionEvent(false);
    CreateResetInfoEvent();

    while (1) {

        monitor_task_set(TASK_MONITOR_BIT_TRACKING);

        uxBits = xEventGroupWaitBits(event_acc_process_sample, 0x01, pdTRUE, pdFALSE, 100);
        if (uxBits == 0) {
            continue;
        }

        incoming_sample_ptr = driver_behaviour_state.current_samples;

        ///////////////////////////
        // process block of data //
        ///////////////////////////

        // handle last 32 samples

        need_sleep = false;

        Calculate_Energy();
        send_acc_sample_to_ble();

        if (driver_behaviour_state.store_calibration) {
            Process_Calibrate();

            continue;
        }

        // avoid fake accident event after calibration
        duration = timeDiff(xTaskGetTickCount(), driver_behaviour_state.block_sample_time) / 1000;
        if (duration <= 3)
            continue;

        // no automatic calibration
        /*
        if (!driver_behaviour_state.calibrated) {
            // Process_Calibrate();
        } else
        */

        {
            switch (driver_behaviour_state.track_state) {
            case TRACKING_STATE_WAKEUP:

                if (ProcessWakeupState()) {
                    driver_behaviour_state.track_state = TRACKING_STATE_ROUTE;

                    set_sleep_timeout_on_ble();

                    buzzer_train(2);

                    DisplayMessage("\r\nStart route\r\n", 0, true);
                    CreateGeneralEvent(0, EVENT_TYPE_START_ROUTE, 1);
                    continue;
                } else {

                    need_sleep = CheckNoActivity();

                    if (need_sleep) {
                        CreateGeneralEvent(LOG_FALSE_WAKEUP, EVENT_TYPE_LOG, 2);
                    }
                }

                break;

            case TRACKING_STATE_ROUTE:
                ProcessDrivingState();

                need_sleep = CheckNoActivity();

                if (need_sleep) {
                    CreateEndRouteEvent();
                    CreateGeneralEvent(LOG_SLEEP_BY_NO_MOVEMENT, EVENT_TYPE_LOG, 2);
                }
                break;

            case TRACKING_STATE_SLEEP:

                duration = timeDiff(xTaskGetTickCount(), driver_behaviour_state.enter_sleeping_time) / 1000;

                if (duration <= 5) {
                    ProcessTamper();
                } else {
                    TestTamperState();

                    ble_services_disconnect();

                    // need after creating end of route immediate events
                    vTaskDelay(5000);

                    DisplayMessageWithTime("Sleep\r\n", 0, true);
                    buzzer_long(1200);

                    configure_acc(Acc_Sleep_Table, ACC_TABLE_SLEEP_SIZE);
                    vTaskDelay(2000);

                    reset_data.reason = RESET_AFTER_SLEEP;

                    isticker_bsp_board_sleep();

                    nrf_gpio_pin_sense_t sense = NRF_GPIO_PIN_SENSE_LOW;
                    nrf_gpio_cfg_sense_set(HAL_LIS3DH_INT2, sense);

                    SleepCPU(true);
                }

                break;
            }
        }

        // ???????????
        // driver_behaviour_state.last_activity_time = xTaskGetTickCount();
        // need_sleep = false;

        if (need_sleep) {
            DisplayMessage("\r\nSleeping...\r\n", 0, true);
            driver_behaviour_state.track_state         = TRACKING_STATE_SLEEP;
            driver_behaviour_state.enter_sleeping_time = xTaskGetTickCount();
            driver_behaviour_state.tamper_sample_count = 0;
        }

        terminal_print_signal_mode();
    }
}

void TestTamperState(void)
{
    uint16_t n;
    uint32_t value;

    n = driver_behaviour_state.tamper_sample_count;

    driver_behaviour_state.tamper_value.X /= n;
    driver_behaviour_state.tamper_value.Y /= n;
    driver_behaviour_state.tamper_value.Z /= n;

    value = GetGValue((AccConvertedSample *)&driver_behaviour_state.tamper_value);

    if (value > 7) {
        // tamper identified
        buzzer_train(9);
        DisplayMessage("\r\nTampered\r\n", 0, true);

        CreateGeneralEvent(LOG_TAMPER, EVENT_TYPE_LOG, 1);

        driver_behaviour_state.tampered   = true;
        driver_behaviour_state.calibrated = false;

        device_config.calibrate_value.avg_value.Z = 0;
        SaveConfiguration(true);

        vTaskDelay(10000);
    }
}

void ProcessTamper(void)
{
    static uint8_t check_flag = false;

    AccSample *sample;
    uint8_t    i;

    if (driver_behaviour_state.tampered)
        return;

    if (!driver_behaviour_state.calibrated)
        return;

    if (driver_behaviour_state.tamper_sample_count == 0) {
        driver_behaviour_state.tamper_value.X = 0;
        driver_behaviour_state.tamper_value.Y = 0;
        driver_behaviour_state.tamper_value.Z = 0;
    }

    for (i = 0; i < SAMPLE_BUFFER_SIZE; i++) {
        sample = (AccSample *)&incoming_sample_ptr[i];

        driver_behaviour_state.tamper_sample_count++;

        driver_behaviour_state.tamper_value.X += sample->X;
        driver_behaviour_state.tamper_value.Y += sample->Y;
        driver_behaviour_state.tamper_value.Z += sample->Z;
    }
}

static void terminal_print_signal_mode(void)
{
    uint8_t len;
    terminal_buffer_lock();

    alert_str[0] = 0;

    switch (driver_behaviour_state.print_signal_mode) {

    case PRINT_SIGNAL_MODE_ENERGY:
        sprintf(alert_str, "\r\nE=%d\r\n", driver_behaviour_state.energy);
        break;

    case PRINT_SIGNAL_MODE_GX:
        sprintf(alert_str, "\r\nGx=%d\r\n", driver_behaviour_state.Gx);
        break;

    case PRINT_SIGNAL_MODE_GY:
        sprintf(alert_str, "\r\nGy=%d\r\n", driver_behaviour_state.Gy);
        break;
    }

    len = strlen(alert_str);

    if (len > 0)
        DisplayMessage(alert_str, len, false);

    terminal_buffer_release();
}

void SleepCPU(bool with_memory_retention)
{

    uint8_t i;

#ifdef BLE_ADVERTISING

    for (i = 0; i < 8; i++) // Retain RAM 0 - RAM 7
    {
        sd_power_ram_power_set(i, (POWER_RAM_POWER_S0POWER_On << POWER_RAM_POWER_S0POWER_Pos) |
                                      (POWER_RAM_POWER_S1POWER_On << POWER_RAM_POWER_S1POWER_Pos) |
                                      (POWER_RAM_POWER_S0RETENTION_On << POWER_RAM_POWER_S0RETENTION_Pos) |
                                      (POWER_RAM_POWER_S1RETENTION_On << POWER_RAM_POWER_S1RETENTION_Pos));
        // if (ret_value != NRF_SUCCESS) {
        //    ret_value = 1;
        //}
    }

#endif

    nrf_power_system_off();

    while (1) {
        nrf_pwr_mgmt_run();
    }
}

bool CheckNoActivity(void)
{
    uint32_t duration;
    int16_t  time_to_sleep;
    bool     need_sleep = false;

#ifdef SLEEP_DISABLE
    return false;
#endif

    uint16_t timeout_in_sec = driver_behaviour_state.sleep_delay_time;

    duration = timeDiff(xTaskGetTickCount(), driver_behaviour_state.last_activity_time) / 1000;

    time_to_sleep                                    = timeout_in_sec - duration;
    driver_behaviour_state.time_to_sleep_left_in_sec = time_to_sleep;

    if (time_to_sleep <= 0) {

        need_sleep = true;
    }

    return need_sleep;
}

void Calculate_Energy(void)
{
    AccSample *sample;
    AccSample *dif_sample;
    uint32_t   delta_energy;
    int32_t    temp;
    uint8_t    i = 0;

    static AccSample prev_sample;

    delta_energy = 0;

    if (driver_behaviour_state.energy < 0) {
        memcpy((unsigned char *)&prev_sample, (unsigned char *)(&incoming_sample_ptr[0]), sizeof(AccSample));
    }

    delta_energy = 0;

    for (i = 0; i < SAMPLE_BUFFER_SIZE; i++) {
        sample = (AccSample *)&incoming_sample_ptr[i];

        temp = (sample->X - prev_sample.X);
        temp *= temp;
        delta_energy += temp;

        temp = (sample->Y - prev_sample.Y);
        temp *= temp;
        delta_energy += temp;

        temp = (sample->Z - prev_sample.Z);
        temp *= temp;
        delta_energy += temp;

        memcpy((unsigned char *)&prev_sample, (unsigned char *)(sample), sizeof(AccSample));
    }

    delta_energy = (uint32_t)sqrt(delta_energy);

    driver_behaviour_state.energy = delta_energy;
}

void ProcessDrivingState(void)
{
    DriverBehaviourState *state = &driver_behaviour_state;
    AccConvertedSample *  sample;

    if (state->energy > GetMinEneryForMovement()) {

        if (driver_behaviour_state.time_to_sleep_left_in_sec < get_current_state_timeout())
            state->last_activity_time = xTaskGetTickCount();
        print_movment();
    }

    for (unsigned char i = 0; i < SAMPLE_BUFFER_SIZE; i++) {
        sample = (AccConvertedSample *)&incoming_sample_ptr[i];
        Process_Accident(state, sample);
    }
}

void send_acc_sample_to_ble(void)
{
    static uint8_t      ble_buffer[16];
    AccConvertedSample *sample;

    if (!ble_services_is_connected())
        return;

    sample = (AccConvertedSample *)&incoming_sample_ptr[0];

    memset(ble_buffer, 0x00, 16);
    memcpy(ble_buffer, sample, sizeof(AccConvertedSample));
    ble_services_notify_acc(ble_buffer, 16);
}

void InitWakeupAlgorithm(void)
{
    driver_behaviour_state.movement_count      = 0;
    driver_behaviour_state.movement_test_count = 0;
}

bool ProcessWakeupState(void)
{
    DriverBehaviourState *state = &driver_behaviour_state;
    AccConvertedSample *  sample;
    bool                  sense;
    bool                  sense1 = false;
    bool                  found  = false;
    EventBits_t           uxBits;
    uint8_t               i;

#ifdef SLEEP_DISABLE
    return true;
#endif

    for (unsigned char i = 0; i < SAMPLE_BUFFER_SIZE; i++) {
        sample = (AccConvertedSample *)&incoming_sample_ptr[i];
        Process_Accident(state, sample);
    }

    if (state->energy > GetMinEneryForMovement()) {
        state->movement_count++;

        if (driver_behaviour_state.time_to_sleep_left_in_sec < get_current_state_timeout())
            driver_behaviour_state.last_activity_time = xTaskGetTickCount();
        print_movment();
    }

    if (state->movement_count >= 7) {
        found = true;
    }

    return found;
}

void Set_Installation_Angle(void)
{
    DriverBehaviourState *state = &driver_behaviour_state;
    float                 x, y, z, angle1;

    x = state->calibrated_value.avg_value.X;
    y = state->calibrated_value.avg_value.Y;
    z = state->calibrated_value.avg_value.Z;

    x /= ACC_NORMALIZATION_VALUE;
    y /= ACC_NORMALIZATION_VALUE;
    z /= ACC_NORMALIZATION_VALUE;

    angle1        = atan(y / z);
    state->angle1 = angle1;
}

uint16_t GetMinEneryForMovement(void)
{
    if (driver_behaviour_state.calibrated)
        return 100;
    else
        return 750;
}

void Process_Calibrate(void)
{
    DriverBehaviourState *state = &driver_behaviour_state;

    float         x, y, z;
    float         angle1, angle2;
    int16_t       temp1, temp2;
    AccSample *   sample;
    unsigned char i;

    for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++) {
        sample = (AccSample *)&incoming_sample_ptr[i];

        state->sum_x += sample->X;
        state->sum_y += sample->Y;
        state->sum_z += sample->Z;
    }

    state->block_count++;

    if (state->block_count >= (3 * 5)) {
        state->block_count *= SAMPLE_BUFFER_SIZE;

        state->sum_x /= state->block_count;
        state->sum_y /= state->block_count;
        state->sum_z /= state->block_count;

        state->calibrated_value.avg_value.X = state->sum_x;
        state->calibrated_value.avg_value.Y = state->sum_y;
        state->calibrated_value.avg_value.Z = state->sum_z;

        x = state->calibrated_value.avg_value.X;
        y = state->calibrated_value.avg_value.Y;
        z = state->calibrated_value.avg_value.Z;

        x /= ACC_NORMALIZATION_VALUE;
        y /= ACC_NORMALIZATION_VALUE;
        z /= ACC_NORMALIZATION_VALUE;

        angle2 = asin(x);
        temp2  = (int16_t)(angle2 * 180 / PI);

        angle1 = atan(y / z);
        temp1  = (int16_t)(angle1 * 180 / PI);

        state->angle1 = angle1;

        if (angle1 < PI * 45 / 180) {
            // y is the driving axis
            state->calibrated_value.axis = 2;
        } else {
            // z is the driving axis
            state->calibrated_value.axis = 3;
        }

        if (driver_behaviour_state.store_calibration) {
            memcpy(&device_config.calibrate_value, &state->calibrated_value, sizeof(CalibratedValue));
            state->tampered = false;
            SaveConfiguration(false);
            driver_behaviour_state.registration_mode = false;
        }

        driver_behaviour_state.store_calibration = false;
        state->block_count                       = 0;
        state->calibrated                        = true;
        buzzer_train(5);

        terminal_buffer_lock();
        sprintf(alert_str + 2, "@?C,%d,%d,%d\r\n", temp1, temp2, state->calibrated_value.axis);
        PostBleAlert(alert_str);
        terminal_buffer_release();

        driver_behaviour_state.block_sample_time = xTaskGetTickCount();
    }
}

void Process_Accident(DriverBehaviourState *state, AccConvertedSample *sample)
{
    unsigned short value;

    if (driver_behaviour_state.print_signal_mode)
        return;

    if (driver_behaviour_state.registration_mode)
        return;

    if (!driver_behaviour_state.calibrated)
        return;

    switch (state->accident_state) {
    case ACCIDENT_STATE_NONE:

        if (abs(sample->turn_direction) >= ACC_MIN_ACCIDENT_VALUE)
            state->accident_state = ACCIDENT_STATE_STARTED;

        if (abs(sample->drive_direction) >= ACC_MIN_ACCIDENT_VALUE)
            state->accident_state = ACCIDENT_STATE_STARTED;

        if (driver_behaviour_state.calibrated) {
            if (abs(sample->earth_direction) >= ACC_MIN_ACCIDENT_VALUE)
                state->accident_state = ACCIDENT_STATE_STARTED;
        }

        if (driver_behaviour_state.record_triggered)
            state->accident_state = ACCIDENT_STATE_STARTED;

        if (state->accident_state == ACCIDENT_STATE_STARTED) {
            state->accident_sample_count = 1;
            state->max_g                 = GetGValue(sample);
            state->sum_g_accident        = state->max_g;
            state->hit_angle             = calculate_accident_hit_angle(sample);
        }

        break;

    case ACCIDENT_STATE_STARTED:

        state->accident_sample_count++;
        value = GetGValue(sample);
        state->sum_g_accident += value;

        if (value > state->max_g) {
            state->max_g                     = value;
            state->sample_in_drive_direction = sample->drive_direction;
            state->sample_in_turn_direction  = sample->turn_direction;
            state->hit_angle                 = calculate_accident_hit_angle(sample);
        }

        if (state->accident_sample_count >= MIN_SAMPLES_FOR_ACCIDENT) {
            if (state->max_g >= (device_config.AccidentG * 10) || driver_behaviour_state.record_triggered) {
                /////////////////////////
                // accident identified //
                /////////////////////////

                FileTransferFailed(true);

                state->last_activity_time               = xTaskGetTickCount();
                state->accident_state                   = ACCIDENT_STATE_IDENTIFIED;
                driver_behaviour_state.record_triggered = false;

                record_trigger(0);
            } else {
                state->accident_state = ACCIDENT_STATE_NONE;
            }
        }

        break;

    case ACCIDENT_STATE_IDENTIFIED:

        driver_behaviour_state.movement_count = 0;

        if (state->accident_sample_count == MIN_SAMPLES_FOR_ACCIDENT) {
            // sending calibrate alert

            terminal_buffer_lock();
            // sprintf(alert_str + 2, "@?X,%d,%d,%d\r\n", state->max_g, state->hit_angle, state->sum_g_accident);
            sprintf(alert_str + 2, "@?X,%d,%d\r\n", state->max_g, state->hit_angle);
            PostBleAlert(alert_str);
            terminal_buffer_release();

            // beep a buzzer
            buzzer_long(4000);
        }

        state->accident_sample_count++;
        // continue recording
        // ..

        // ~30 seconds delay between accident
        if (state->accident_sample_count >= DELAY_BETWEEN_ACCIDENTS) {

            state->accident_state = ACCIDENT_STATE_NONE;
            DisplayMessage("\r\nListen to accident on\r\n", 0, true);
        }
        break;
    }
}

signed short calculate_accident_hit_angle(AccConvertedSample *sample)
{
    float        value;
    signed short v1;

    if (sample->drive_direction == 0)
        sample->drive_direction = 1;

    value = atan((float)sample->turn_direction / sample->drive_direction);
    value = value * 180 / PI;

    if (sample->drive_direction < 0) {
        if (sample->turn_direction > 0)
            value = 360 + value;
    } else {
        value = 180 + value;
    }

    v1 = (short)value;

    /*
        // patch (reverse direction)
        v1 -= 180;
        if (v1 < 0)
            v1 += 360;
     */

    return v1;
}

unsigned short GetGValue(AccConvertedSample *sample)
{
    unsigned int value;

    value = sample->drive_direction * sample->drive_direction;
    value += sample->turn_direction * sample->turn_direction;
    value += sample->earth_direction * sample->earth_direction;

    value = sqrt(value);
    return value;
}

bool IsDeviceMoved(unsigned moved_in_last_seconds)
{
    uint32_t duration;

    duration = timeDiff(xTaskGetTickCount(), driver_behaviour_state.last_activity_time) / 1000;

    if (duration <= moved_in_last_seconds)
        return true;
    else
        return false;
}

void set_sleep_timeout(uint16_t value)
{
    if (value > driver_behaviour_state.sleep_delay_time || !driver_behaviour_state.manual_delayed)
        driver_behaviour_state.sleep_delay_time = value;
}

uint16_t get_current_state_timeout(void)
{
    uint16_t timeout_value;

    if (ble_services_is_connected()) {
        if (driver_behaviour_state.track_state == TRACKING_STATE_WAKEUP)
            timeout_value = SLEEP_TIMEOUT_ON_WAKEUP_BLE_CONNECTED;
        else
            timeout_value = SLEEP_TIMEOUT_ON_ROUTE_BLE_CONNECTED;

    } else {
        if (driver_behaviour_state.track_state == TRACKING_STATE_WAKEUP)
            timeout_value = SLEEP_TIMEOUT_ON_WAKEUP_BLE_DISCONNECTED;
        else
            timeout_value = SLEEP_TIMEOUT_ON_ROUTE_BLE_DISCONNECTED;
    }

    return timeout_value;
}

void set_sleep_timeout_on_ble(void)
{
    uint16_t timeout_value = get_current_state_timeout();

    set_sleep_timeout(timeout_value);
}

void print_movment(void)
{
    static uint32_t time = 0;
    uint32_t        duration;

    duration = timeDiff(xTaskGetTickCount(), time);

    if (duration > 1000) {
        DisplayMessage("\r\n*\r\n", 0, true);
        time = xTaskGetTickCount();
    }
}