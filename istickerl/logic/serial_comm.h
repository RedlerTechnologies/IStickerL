#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void serial_comm_init(void);
void serial_comm_send_text(char tx_data[]);
void serial_comm_process_rx(void);