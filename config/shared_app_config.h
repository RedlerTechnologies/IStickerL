#pragma once

#define BLE_ADVERTISING
#define FLASH_TEST_ENABLE   0


#define NRFX_LOG_LEVEL_OFF 0
#define NRFX_LOG_LEVEL_ERROR 1
#define NRFX_LOG_LEVEL_WARNING 2
#define NRFX_LOG_LEVEL_INFO 3
#define NRFX_LOG_LEVEL_DEBUG 4

// <<< Use Configuration Wizard in Context Menu >>>\n

// <h> Cloud-Wise

// <o> CLOUD_WISE_DEFAULT_LOG_LEVEL - Global Default Log level

// <0=> Off
// <1=> Error
// <2=> Warning
// <3=> Info
// <4=> Debug

#ifndef CLOUD_WISE_DEFAULT_LOG_LEVEL
#define CLOUD_WISE_DEFAULT_LOG_LEVEL NRFX_LOG_LEVEL_INFO
#endif

// <<< end of configuration section >>>

#if defined(BOARD_PCA10040)

#elif defined(BOARD_CUSTOM)

#if BOARD_CUSTOM == ISTICKERL_REV_1

#else

#error "Cloud-Wise IStickerL Custom Board is unknown " ## BOARD_CUSTOM

#endif

#endif


//===============================================

// <e> RNG_ENABLED - nrf_drv_rng - RNG peripheral driver - legacy layer
//==========================================================
