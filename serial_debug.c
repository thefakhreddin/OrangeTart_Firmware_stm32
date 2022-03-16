#include "serial_debug.h"
#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "stdarg.h"
#include "stdio.h"

#define BUFFER_SIZE 80

void print(char *msg, ...)
{
	char buffer[BUFFER_SIZE] = {0};
	
	va_list args;
	va_start(args, msg);
	vsprintf(buffer, msg, args);
	
	/*DMA1_Channel2->CNDTR = BUFFER_SIZE;
	DMA1_Channel2->CCR |= DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE | DMA_CCR_CIRC;
	
	DMA1_Channel2->CPAR = (uint32_t) (&(USART1->TDR));
	DMA1_Channel2->CMAR = (uint32_t) (buffer);
	
	DMA1_Channel2->CCR |= DMA_CCR_EN;
	
	while ((DMA1->ISR & DMA_ISR_TCIF2) != DMA_ISR_TCIF2);
	
	DMA1->IFCR = DMA_IFCR_CTCIF2;
	DMA1_Channel2->CCR &= ~DMA_CCR_EN;*/
	
	for (uint8_t i = 0; i < BUFFER_SIZE; i++)
	{
		USART1->TDR = buffer[i];
		while ((USART1->ISR & USART_ISR_TXE) != USART_ISR_TXE);
	}
}
