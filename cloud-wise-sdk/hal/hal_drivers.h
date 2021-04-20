#pragma once

#include "nrfx_pwm.h"
#include "nrfx_spi.h"
#include "nrfx_twim.h"
#include "nrfx_uart.h"

extern const nrfx_spi_t * hal_flash_spi;
extern const nrfx_twim_t *hal_lis3dh_twi;
extern const nrfx_uart_t *hal_uart;
extern const nrfx_pwm_t * hal_buzzer;

#define HAL_SAADC_VBAT_CHANNEL 0