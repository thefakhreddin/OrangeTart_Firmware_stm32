#include "buzzer.h"
#include "config.h"
#include "TIM.h"

void Buzzer_Init(void)
{
	// ##### TIM14 (Buzzer) #####
	RCC->APB1ENR |= RCC_APB1ENR_TIM14EN; /* Enable TIM14 clock */
	
	TIM14->CR1 |= TIM_CR1_ARPE; /* Auto-reload preload enable */
	TIM14->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
	TIM14->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1PE; /* Toggle mode 1 and Preload enable */
	//TIM14->CCER |= TIM_CCER_CC1E; /* Capture/Compare 1 output enable */
	TIM14->EGR |= TIM_EGR_UG; /* Update generation */
	//TIM14->CR1 |= TIM_CR1_CEN; /* Enable timer */
}

void Buzzer_Tone(uint16_t note, uint32_t duration)
{
	TIM14->PSC = 31;
	TIM14->ARR = 32000000UL / ((31 + 1) * (note + 1)) / 2;
	TIM14->CCER |= TIM_CCER_CC1E; /* Capture/Compare 1 output enable */
	TIM14->EGR |= TIM_EGR_UG; /* Update generation */
	TIM14->CR1 |= TIM_CR1_CEN; /* Enable timer */
	TIM_Delay(duration);
	TIM14->CCER &= ~TIM_CCER_CC1E; /* Capture/Compare 1 output disable */
	TIM14->CR1 &= ~TIM_CR1_CEN; /* Enable timer */
}

void Buzzer_ToneStart(uint16_t note)
{
	TIM14->PSC = 31;
	TIM14->ARR = 32000000UL / ((31 + 1) * (note + 1)) / 2;
	TIM14->CCER |= TIM_CCER_CC1E; /* Capture/Compare 1 output enable */
	TIM14->EGR |= TIM_EGR_UG; /* Update generation */
	TIM14->CR1 |= TIM_CR1_CEN; /* Enable timer */
}

void Buzzer_ToneStop(void)
{
	TIM14->CCER &= ~TIM_CCER_CC1E; /* Capture/Compare 1 output disable */
	TIM14->CR1 &= ~TIM_CR1_CEN; /* Enable timer */
}
