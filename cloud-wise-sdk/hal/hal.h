#pragma once

#include "hal_data_types.h"

typedef enum {
    HAL_EVENT_NOTHING,

    HAL_EVENT_LIS3DH_INT1,
    HAL_EVENT_LIS3DH_INT2,

    HAL_EVENT_UART0_RX,
    HAL_EVENT_UART0_TX,
} hal_event_type_t;

typedef void (*hal_evt_handler_t)(const hal_event_type_t event);

void hal_init(hal_evt_handler_t evt_handler, nrfx_spi_evt_handler_t spi_handler);

uint32_t hal_read_device_serial_number(char *const p_serial, uint8_t max_len);

void hal_interrupts_set(bool enable_int1, bool enable_int2);

uint8_t hal_scan_twim0(void);

int16_t hal_read_vdd_raw(void);

void isticker_bsp_board_sleep(void);
