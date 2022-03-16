#include "TIM.h"
#include "GPIO.h"

volatile uint32_t ms_ticks = 0;

void SysTick_Handler(void)
{
	ms_ticks++;
}

void TIM_Delay(uint32_t ms)
{
	/* Using systick */
	uint32_t s = ms_ticks;
	while (ms_ticks - s < ms);
}
