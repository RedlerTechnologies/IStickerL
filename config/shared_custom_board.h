#pragma once

#ifdef BOARD_CUSTOM

#if BOARD_CUSTOM == ISTICKERL_REV_1

#define LEDS_NUMBER 0
#define LEDS_ACTIVE_STATE 0

#define BUTTONS_NUMBER 0
#define BUTTONS_ACTIVE_STATE 0

#else

#error "Cloud-Wise IStickerL Custom Board is unknown " ## BOARD_CUSTOM

#endif

#endif
