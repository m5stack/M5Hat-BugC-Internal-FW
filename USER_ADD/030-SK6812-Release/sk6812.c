#include "tim.h"
#include "sk6812.h"
#include <stdlib.h>

uint8_t led_num;
uint32_t *color_buf = NULL;
void color_set_single(uint32_t color);

#define gpio_low() GPIOA->BRR = GPIO_PIN_0
#define gpio_high() GPIOA->BSRR = GPIO_PIN_0
//#define delay_150ns() __nop(); __nop(); __nop(); __nop(); __nop(); __nop()		//64MHZ
//#define delay_300ns() delay_150ns(); __nop(); __nop(); __nop(); __nop(); __nop()

#define delay_150ns() __nop(); __nop()
#define delay_300ns() delay_150ns();__nop(); __nop(); __nop(); __nop(); __nop(); __nop(); __nop()
#define delay_600ns() delay_300ns(); delay_300ns(); __nop(); __nop(); __nop(); __nop(); __nop(); __nop()
#define delay_900ns() delay_600ns(); delay_300ns()

#define out_bit_low() \
  gpio_high(); \
  delay_300ns(); \
  gpio_low(); \
  delay_900ns()

#define out_bit_high() \
  gpio_high(); \
  delay_600ns(); \
  gpio_low(); \
  delay_600ns()
//inline 
#define restart() \
 do{ \
  for (uint8_t i = 0; i < 133; i++) {	\
    delay_600ns();\
  }\
 } while(0)

void GPIO_init() {
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}


void sk6812_init(uint8_t num) {
  GPIO_init();
	restart();
  color_buf = (uint32_t *)calloc(num, sizeof(uint32_t));
  led_num = num;
}

void neopixel_set_color(uint8_t num, uint32_t color) {
  color_buf[num] = color;
}

void neopixel_set_all_color(uint32_t color) {
  for (uint8_t i = 0; i < led_num; i++) {
    color_buf[i] = color;
  }
}

void neopixel_show(void) {

  __disable_irq();
  for (uint8_t i = 0; i < led_num; i++) {
    color_set_single(color_buf[i]);
  }
  __enable_irq();
  restart();
}

void color_set_single(uint32_t color) {
  for (uint8_t i = 0; i < 24; i++) {
    if (color & (1 << (23 - i))) {
      out_bit_high();
    }
    else {
      out_bit_low();
    }
  }
}

