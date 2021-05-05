#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

extern uint8_t alert_str[];


void serial_comm_init(void);
void serial_comm_send_text(void);
void serial_comm_process_rx(void);

void DisplayMessage( uint8_t* message, uint8_t size, bool with_lock);
void DisplayMessageWithTime(uint8_t *message, uint8_t len, bool with_lock);
//void DisplayMessageWithNoLock(uint8_t *message, uint8_t len);

void terminal_buffer_lock(void);
void terminal_buffer_release(void);