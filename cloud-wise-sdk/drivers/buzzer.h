#pragma once

#include "hal/hal_data_types.h"

bool buzzer_init(void);

bool buzzer_train(uint8_t repeats);
bool buzzer_long(uint16_t time_ms);
bool buzzer_end(void);
