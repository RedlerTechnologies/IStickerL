#include "events.h"

#include "FreeRTOS.h"
#include "ble/ble_services_manager.h"
#include "commands.h"
#include "configuration.h"
#include "drivers/buzzer.h"
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

xSemaphoreHandle event_semaphore;

static uint32_t event_id = 0;

bool CreateEvent(IStickerEvent *event)
{
    if (xSemaphoreTake(event_semaphore, 100) == 0)
        return false;

    // ????????? if (!event->immediate_event)
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

void CreateAccidentEvent(void)
{
    IStickerEvent accident_event;
    uint8_t       buffer[8];

    memset(buffer, 0x00, 8);
    buffer[0] = driver_behaviour_state.max_g / 10;
    buffer[1] = driver_behaviour_state.hit_angle / 2;

    memcpy(buffer + 4, (uint8_t*)&acc_record.record_id, 4);

    accident_event.time       = 0;
    accident_event.data_len   = 8;
    accident_event.data       = buffer;
    accident_event.event_type = EVENT_TYPE_ACCIDENT;

    CreateEvent(&accident_event);
}

void CreateGeneralEvent(uint32_t value, uint8_t event_type, uint8_t value_size)
{
    IStickerEvent accident_event;
    uint8_t       buffer[4];

    memset(buffer, 0x00, 4);
    memcpy(buffer, (uint8_t *)(&value), value_size);

    accident_event.time       = 0;
    accident_event.data_len   = value_size;
    accident_event.data       = buffer;
    accident_event.event_type = event_type;

    CreateEvent(&accident_event);
}

void CreateEndRouteEvent(void)
{
    IStickerEvent accident_event;
    uint8_t       buffer[13];

    // 0 - stop bits
    // 1-4 - distance
    // 5-8 - duration
    // 9-12 - odometer

    memset(buffer, 0x00, 13);

    accident_event.time       = 0;
    accident_event.data_len   = 13;
    accident_event.data       = buffer;
    accident_event.event_type = EVENT_TYPE_END_ROUTE;

    CreateEvent(&accident_event);
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