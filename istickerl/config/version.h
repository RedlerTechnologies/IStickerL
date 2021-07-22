#pragma once

// Name of device (Will be included in the advertising data)
#define DEVICE_NAME "IStickerL"
extern char version[];

// Manufacturer Name String (Device Information Service)
#define MANUFACTURER_NAME "Cloud-Wise"

// Model Number String (Device Information Service)
#if defined(BOARD_PCA10040)
#define MODEL_NUM "DevKit"
#define HARDWARE_REV "1.0"

#elif defined(BOARD_CUSTOM)

#define MODEL_NUM STRINGIFY(BOARD_CUSTOM)

#if BOARD_CUSTOM == ISTICKERL_REV_1

#define HARDWARE_TYPE 11
#define HARDWARE_REV STRINGIFY(HARDWARE_TYPE)

#else

#error "Cloud-Wise IStickerL Custom Board is unknown " ## BOARD_CUSTOM

#endif

#else

#error "Unknown MODEL_NUM"
#endif

#define APP_MAJOR_VERSION 1
#define APP_MINOR_VERSION 2
#define APP_BUILD 17

// Firmware Revision String (Device Information Service)
#define FIRMWARE_REV STRINGIFY(APP_MAJOR_VERSION) "." STRINGIFY(APP_MINOR_VERSION) "." STRINGIFY(APP_BUILD)
