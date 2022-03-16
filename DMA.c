#include "DMA.h"

void DMA_Init(void)
{
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
}
