#ifndef WS2812B_H
#define WS2812B_H

#include "stm32f030x6.h"


typedef enum
{
  LED_ALL, LED_1, LED_2, LED_3, LED_4, LED_5, LED_6
} LEDIndex;

typedef enum
{
  OFF, WHITE, PURPLE, BLUE, GREEN, YELLOW, ORANGE, RED, PINK
} LEDColor;


void WS2812B_Init(void);
void WS_Queue(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
void WS_QueueSimple(LEDIndex led_index, LEDColor led_color);
void WS_Send(void);

#endif // WS2812B_H
