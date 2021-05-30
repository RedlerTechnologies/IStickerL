#include "events.h"

#include "FreeRTOS.h"
#include "ble/ble_services_manager.h"
#include "commands.h"
#include "configuration.h"
#include "decoder.h"
#include "drivers/buzzer.h"
#include "drivers/flash.h"
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

extern xSemaphoreHandle     clock_semaphore;
extern Calendar             absolute_time;
extern DriverBehaviourState driver_behaviour_state;
extern AccRecord            acc_record;
extern ResetData            reset_data_copy;

xSemaphoreHandle event_semaphore;

extern ScanResult scan_result;

// static uint32_t event_id = 0;

/*
bool CreateEvent(IStickerEvent *event)
{
    if (xSemaphoreTake(event_semaphore, 100) == 0)
        return false;

    // if (!event->immediate_event)
    if (false)
        event_id++;

    static uint8_t ble_buffer[64];
    uint16_t       len, crc;
    uint8_t        ptr = 2;

    memset(ble_buffer, 0x00, 64);

    // record header
    ble_buffer[ptr]     = 'D';
    ble_buffer[ptr + 1] = '0';

    ptr += 2;

    // record length
    len = event->data_len + 17;
    memcpy(ble_buffer + ptr, (uint8_t *)&len, 2);
    ptr += 2;

    // record time

    if (event->time == 0) {
        xSemaphoreTake(clock_semaphore, portMAX_DELAY);
        event->time = GetTimeStampFromDate();
        xSemaphoreGive(clock_semaphore);
    }

    memcpy(ble_buffer + ptr, (uint8_t *)&event->time, 4);
    ptr += 4;

    // event time status
    ptr++;

    // event id
    memcpy(ble_buffer + ptr, (uint8_t *)&event_id, 4);
    ptr += 4;

    // event type
    memcpy(ble_buffer + ptr, (uint8_t *)&event->event_type, 2);
    ptr += 2;

    // event data
    memcpy(ble_buffer + ptr, (uint8_t *)event->data, event->data_len);
    ptr += event->data_len;

    // crc16
    crc = CRC16_Calc(ble_buffer + 2, ptr - 2, 0);
    memcpy(ble_buffer + ptr, (uint8_t *)&crc, 2);
    ptr += 2;

    // write to flash here
    // ..

    ble_buffer[0] = 0x80;
    ble_buffer[1] = ptr - 2;
    ble_services_notify_event(ble_buffer, ptr);

    xSemaphoreGive(event_semaphore);
}
*/

static void init_event(IStickerEvent *event)
{
    uint8_t ch = 0x00;
    memset(event, ch, sizeof(IStickerEvent));
}

bool CreateEvent(IStickerEvent *event)
{
    static uint8_t ble_buffer[64];
    uint32_t       eid = 0;
    uint16_t       len, crc;
    uint8_t        ptr = 4;
    bool           result;
    bool           drop            = false;
    bool           semaphore_taken = false;
    bool           save_in_flash   = true;

    // return true; 

    if (driver_behaviour_state.block_new_events)
        drop = true;

    if (xSemaphoreTake(event_semaphore, 500) == 0)
        drop = true;
    else
        semaphore_taken = true;

    if (!IsDBInitialized())
        drop = true;

    if (drop) {
        if (semaphore_taken)
            xSemaphoreGive(event_semaphore);

        DisplayMessage("\r\nDrop event\r\n", 0, true);

        return false;
    }

    if (event->immediate_event) {
        save_in_flash = event->save_in_flash;
    }

    if (save_in_flash)
        eid = scan_result.write_marker.event_id;

    memset(ble_buffer, 0x00, 64);

    // record header
    ble_buffer[ptr]     = 'D';
    ble_buffer[ptr + 1] = '0';

    ptr += 2;

    // record length
    len = event->data_len + 17;
    memcpy(ble_buffer + ptr, (uint8_t *)&len, 2);
    ptr += 2;

    // record time

    if (event->time == 0) {
        xSemaphoreTake(clock_semaphore, portMAX_DELAY);
        event->time = GetTimeStampFromDate();
        xSemaphoreGive(clock_semaphore);
    }

    memcpy(ble_buffer + ptr, (uint8_t *)&event->time, 4);
    ptr += 4;

    // event time status
    ptr++;

    // event id
    memcpy(ble_buffer + ptr, (uint8_t *)&eid, 4);
    ptr += 4;

    // event type
    memcpy(ble_buffer + ptr, (uint8_t *)&event->event_type, 2);
    ptr += 2;

    // event data
    memcpy(ble_buffer + ptr, (uint8_t *)event->data, event->data_len);
    ptr += event->data_len;

    // crc16
    crc = CRC16_Calc(ble_buffer + 4, ptr - 4, 0);
    memcpy(ble_buffer + ptr, (uint8_t *)&crc, 2);
    ptr += 2;

    // write to flash here
    if (save_in_flash)
        result = flash_save_event(ble_buffer, eid, event->event_type, ptr - 4);
    else {
        terminal_buffer_lock();
        sprintf(alert_str, "immediate event: type=%d\r\n", event->event_type);
        DisplayMessage(alert_str, 0, false);
        terminal_buffer_release();
    }

    if (event->immediate_event) {
        ble_buffer[2] = 0x80;
        ble_buffer[3] = ptr - 4;
        ble_services_notify_event(ble_buffer + 2, ptr - 2);
    }

    if (result)
        scan_result.write_marker.event_id++;

    xSemaphoreGive(event_semaphore);

    return result;
}

void CreateAccidentEvent(void)
{
    IStickerEvent accident_event;
    uint8_t       buffer[8];

    init_event(&accident_event);

    memset(buffer, 0x00, 8);
    buffer[0] = driver_behaviour_state.max_g / 10;
    buffer[1] = driver_behaviour_state.hit_angle / 2;

    memcpy(buffer + 4, (uint8_t *)&acc_record.record_id, 4);

    accident_event.data_len        = 8;
    accident_event.data            = buffer;
    accident_event.event_type      = EVENT_TYPE_ACCIDENT;
    accident_event.immediate_event = true;
    accident_event.save_in_flash   = true;

    CreateEvent(&accident_event);
}

void CreateGeneralEvent(uint32_t value, uint8_t event_type, uint8_t value_size)
{
    IStickerEvent event;
    uint8_t       buffer[4];

    init_event(&event);

    memset(buffer, 0x00, 4);
    memcpy(buffer, (uint8_t *)(&value), value_size);

    event.data_len   = value_size;
    event.data       = buffer;
    event.event_type = event_type;

    CreateEvent(&event);
}

void CreateDebugEvent(uint16_t value_type, int32_t value, bool extened)
{
    IStickerEvent event;
    uint8_t       buffer[8];
    uint8_t       type_size;
    uint8_t       value_len;

    init_event(&event);
    memset(buffer, 0x00, 8);

    if (extened) {
        event.event_type = EVENT_TYPE_DEBUG_EXT;
        value_len        = 4;
        type_size        = 1;
    } else {
        event.event_type = EVENT_TYPE_DEBUG;
        value_len        = 2;
        type_size        = 2;
    }

    event.data_len = (type_size + value_len);
    event.data     = buffer;

    memset(buffer, 0x00, 8);
    memcpy(buffer, &value_type, type_size);
    memcpy(buffer + type_size, &value, value_len);

    CreateEvent(&event);
}

void CreateEndRouteEvent(void)
{
    IStickerEvent event;
    uint8_t       buffer[13];

    init_event(&event);

    // 0 - stop bits
    // 1-4 - distance
    // 5-8 - duration
    // 9-12 - odometer

    memset(buffer, 0x00, 13);

    event.data_len   = 13;
    event.data       = buffer;
    event.event_type = EVENT_TYPE_END_ROUTE;

    CreateEvent(&event);
}

void CreateVersionEvent(bool is_immediate)
{
    IStickerEvent event;
    uint8_t       buffer[4];

    init_event(&event);

    buffer[0] = HARDWARE_TYPE;
    buffer[1] = APP_MAJOR_VERSION;
    buffer[2] = APP_MINOR_VERSION;
    buffer[3] = APP_BUILD;

    event.data_len        = 4;
    event.data            = buffer;
    event.immediate_event = is_immediate;

    if (is_immediate) {
        event.event_type = EVENT_TYPE_KEEP_ALIVE;
    } else {
        event.event_type = EVENT_TYPE_VERSION;
    }

    CreateEvent(&event);
}

void CreateResetInfoEvent(void)
{
    IStickerEvent event;
    uint8_t       buffer[13];

    init_event(&event);

    memcpy(buffer, &reset_data_copy.reason, 1);
    memcpy(buffer + 1, &reset_data_copy.v1, 4);
    memcpy(buffer + 5, &reset_data_copy.v2, 4);
    memcpy(buffer + 9, &reset_data_copy.v3, 4);

    event.data_len   = 13;
    event.data       = buffer;
    event.event_type = EVENT_TYPE_RESET_INFO;

    CreateEvent(&event);
}

void CreateMeasurementEvent(float bat_voltage, float temperature)
{
    static uint8_t  buffer[25];
    static uint32_t time     = 0;
    uint32_t        duration = 0;
    IStickerEvent   event;

    init_event(&event);

    duration = timeDiff(xTaskGetTickCount(), time) / 1000;

    event.immediate_event = true;

    if (duration > (10 * 60)) {
        event.save_in_flash = true;
        time                = xTaskGetTickCount();
    }

    memset(buffer, 0x00, 25);

    buffer[0] = 0x3F;
    memcpy(buffer + 13, &temperature, 4);
    memcpy(buffer + 17, &bat_voltage, 4);

    event.data_len   = 25;
    event.data       = buffer;
    event.event_type = EVENT_TYPE_MEASURE;

    CreateEvent(&event);
}

uint16_t CRC16_Calc(uint8_t *ptrPct, uint16_t pctLen, uint16_t crc16)
{
    static uint8_t OddParity[16] = {0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0};
    uint16_t       pwr = 0, data = 0;

    // if (pctLen >= 256)
    // 	return 0;

    if (pctLen > 1024)
        return 0;

    for (pwr = 0; pwr < pctLen; pwr++) {
        data = (uint16_t)ptrPct[pwr];
        data = ((data ^ (crc16 & 0xFF)) & 0xFF);
        crc16 >>= 8;

        if (OddParity[data & 0xF] ^ OddParity[data >> 4])
            crc16 ^= 0xC001;

        data <<= 6;
        crc16 ^= data;
        data <<= 1;
        crc16 ^= data;
    }

    return (crc16);
}