#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void serial_comm_init(void);
void serial_comm_send_text(void);
void serial_comm_process_rx(void);

void DisplayMessage( uint8_t* message, uint8_t size);
void DisplayMessageWithTime(uint8_t *message, uint8_t len);
void DisplayMessageWithNoLock(uint8_t *message, uint8_t len);