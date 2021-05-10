#pragma once

#include "boards.h"

#define APP_MAJOR_VERSION   1
#define APP_MINOR_VERSION   1
#define APP_BUILD           9

#define HAL_UICR_DEVICE_SERIAL_NUMBER 0

#if defined(BOARD_PCA10040)

#error "Cloud-Wise IStickerL not configured for DevKit"

#elif defined(BOARD_CUSTOM)

#if BOARD_CUSTOM == ISTICKERL_REV_1

#define HAL_LED_GREEN NRF_GPIO_PIN_MAP(0, 28)
#define HAL_LED_RED NRF_GPIO_PIN_MAP(0, 27)

#define HAL_USART0_TX NRF_GPIO_PIN_MAP(0, 24)
#define HAL_USART0_RX NRF_GPIO_PIN_MAP(0, 23)

#define HAL_TWIM0_SDA NRF_GPIO_PIN_MAP(0, 19)
#define HAL_TWIM0_SCL NRF_GPIO_PIN_MAP(0, 20)
#define HAL_TWIM0_FREQ NRF_TWIM_FREQ_400K

#define HAL_LIS3DH_INT1 NRF_GPIO_PIN_MAP(0, 25)
#define HAL_LIS3DH_INT2 NRF_GPIO_PIN_MAP(0, 26)

#define HAL_SPI1_CLK NRF_GPIO_PIN_MAP(0, 14)
#define HAL_SPI1_MOSI NRF_GPIO_PIN_MAP(0, 12)
#define HAL_SPI1_MISO NRF_GPIO_PIN_MAP(0, 13)
#define HAL_SPI1_SS NRF_GPIO_PIN_MAP(0, 11)

#define HAL_SPI_FLASH_RESETN NRF_GPIO_PIN_MAP(0, 9)

#define HAL_BUZZER_PWM NRF_GPIO_PIN_MAP(0, 6)
#define HAL_BUZZER_PWM_CYCLE_COUNT 2000

#define HAL_PDM_CLK NRF_GPIO_PIN_MAP(0, 7)
#define HAL_PDM_DIN NRF_GPIO_PIN_MAP(0, 8)

#define HAL_ADC_VBATT NRF_SAADC_INPUT_VDD

#define timeDiff(a,b)    ( (int32_t)(a) - (int32_t)(b) )

#else

#error "Cloud-Wise IStickerL Custom Board is unknown " ## BOARD_CUSTOM

#endif

#else
#error "Cloud-Wise IStickerL Board is not defined"
#endif
