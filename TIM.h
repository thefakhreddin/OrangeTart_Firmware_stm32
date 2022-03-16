#ifndef TIM_H
#define TIM_H

#include "stm32f030x6.h"

extern volatile uint32_t ms_ticks;

void TIM_Delay(uint32_t ms);

#endif // TIM_H
