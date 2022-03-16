#include "RCC.h"

void RCC_Init(void)
{
	/* 32MHz HSI PLL Configuration */
	RCC->CR &= ~RCC_CR_PLLON; /* Keep PLL off */
	
	RCC->CFGR &= ~RCC_CFGR_PLLSRC_Msk;
	RCC->CFGR |= RCC_CFGR_PLLSRC_HSI_DIV2;
	
	RCC->CFGR &= ~RCC_CFGR_PLLMUL_Msk;
	RCC->CFGR |= RCC_CFGR_PLLMUL8;
	
	RCC->CR |= RCC_CR_PLLON; /* Turn PLL on */
	while ((RCC->CR & RCC_CR_PLLRDY) == 0); /* Wait for PLL to be ready */
	
	RCC->CFGR &= ~RCC_CFGR_HPRE_Msk; /* Clear AHB resister */
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1; /* AHB Prescaler */
	
	RCC->CFGR &= ~RCC_CFGR_PPRE_Msk; /* Clear APB1 resister */
	RCC->CFGR |= RCC_CFGR_PPRE_DIV1; /* APB1 Prescaler */
	
	RCC->CFGR &= ~RCC_CFGR_SW_Msk; /* Clear SW resister */
	RCC->CFGR |= RCC_CFGR_SW_PLL; /* Set PLL as system clock */
}
