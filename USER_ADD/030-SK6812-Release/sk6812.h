#pragma once

#include "stdint.h"

void sk6812_init(uint8_t num);
void neopixel_show(void);
void neopixel_set_color(uint8_t num, uint32_t color);
void neopixel_set_all_color(uint32_t color);


