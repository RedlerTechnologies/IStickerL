#include "events.h"

#include "../ble/ble_services_manager.h"
#include "../drivers/buzzer.h"
#include "../logic/serial_comm.h"
#include "Configuration.h"
#include "FreeRTOS.h"
#include "TrackingAlgorithm.h"
#include "commands.h"
#include "hal/hal_boards.h"
#include "hal/hal_drivers.h"
#include "semphr.h"
#include "task.h"

#include <stdlib.h>
#include <string.h>

static uint32_t event_id = 0;

void CreateEvent(IStickerEvent *event)
{
    // need semaphore on this fucktion ..
    // ..

    event_id++;

    static uint8_t ble_buffer[64];
    uint16_t       len, crc;
    uint8_t        ptr = 0;

    memset(ble_buffer, 0x00, 64);

    // record header
    ble_buffer[ptr]     = 'D';
    ble_buffer[ptr + 1] = '0';

    ptr += 2;

    // record length
    len = event->data_len + 12;
    memcpy(ble_buffer + ptr, (uint8_t *)&len, 2);
    ptr += 2;

    // record time
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
    memcpy(ble_buffer + ptr, (uint8_t *)&event->data, event->data_len);
    ptr += event->data_len;

    // crc16
    crc = CRC16_Calc(ble_buffer, ptr - 2, 0);

    // write to flash here
    // ..

    ble_services_update_event(ble_buffer, ptr);
}

void CreateAccidentEvent(void)
{
    DriverBehaviourState *driver_behaviour_state;

    static IStickerEvent accident_event;
    static uint8_t       buffer[8];

    memcpy(buffer, (uint8_t *)&driver_behaviour_state->max_g, 2);

    accident_event.time       = 0; // ?????????????
    accident_event.data_len   = 8;
    accident_event.data       = buffer;
    accident_event.event_type = 16; // accident type

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