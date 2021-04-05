#pragma once

#include "hal/hal_data_types.h"

typedef struct
{
  uint32_t time;
  uint16_t data_len;
  uint16_t event_type;
  uint8_t* data;
}
IStickerEvent; 


void CreateEvent(IStickerEvent *event);
void CreateAccidentEvent(void);

uint16_t CRC16_Calc(uint8_t * ptrPct, uint16_t pctLen, uint16_t  crc16);