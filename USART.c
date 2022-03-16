#include "USART.h"
#include "GPIO.h"
#include "TIM.h"


volatile uint8_t tx_busy = 0;

volatile uint8_t buffer_out[PACKET_SIZE] = {0};
volatile uint8_t buffer_in[BUFFER_SIZE] = {0};
static uint8_t buffer_in_pos = 0;
volatile static uint8_t time_out = 0;

void USART1_IRQHandler(void)
{		
	/* Interrupt fired by RXNE flag */
	if ((USART1->ISR & USART_ISR_RXNE) == USART_ISR_RXNE)
	{
		/*if (buffer_in_index >= PACKET_SIZE)
		{
			for (uint8_t i = 0; i <= buffer_in_index; i++)
			{
				buffer_in[i] = 0;
			}
			buffer_in_index = 0;
			return;
		}
		buffer_in[buffer_in_index++] = USART1->RDR;*/
	}
	/* Interrupt fired by TC flag */
	if ((USART1->ISR & USART_ISR_TC) == USART_ISR_TC)
	{
		USART1->ICR |= USART_ICR_TCCF;
	}
	/* Interrupt fired by RTOF flag */
	if ((USART1->ISR & USART_ISR_RTOF) == USART_ISR_RTOF)
	{
		USART1->ICR |= USART_ICR_RTOCF;
		if (DMA1_Channel5->CNDTR % PACKET_SIZE)
		{
			DMA1_Channel5->CCR &= ~DMA_CCR_EN; /* Disable DMA1 channel 5*/
			DMA1_Channel5->CNDTR = sizeof(buffer_in) / sizeof(uint8_t);
			DMA1_Channel5->CCR |= DMA_CCR_EN; /* (8) */
		}
	}
}

void DMA1_Channel4_5_IRQHandler(void)
{
	if ((DMA1->ISR & DMA_ISR_TCIF4) == DMA_ISR_TCIF4)
	{
		DMA1->IFCR |= DMA_IFCR_CTCIF4 /* Clear channel 4 interrupt flag */;
		DMA1_Channel4->CCR &= ~DMA_CCR_EN; /* Disable DMA1 channel 4*/
		tx_busy = 0;
	}
	if ((DMA1->ISR & DMA_ISR_HTIF5) == DMA_ISR_HTIF5)
	{
		DMA1->IFCR |= DMA_IFCR_CHTIF5; /* Clear channel 5 HT flag */
		buffer_in[(BUFFER_SIZE - 1) / 2] = 1; /* Set unred packet flag */
	}
	if ((DMA1->ISR & DMA_ISR_TCIF5) == DMA_ISR_TCIF5)
	{
		DMA1->IFCR |= DMA_IFCR_CTCIF5; /* Clear channel 5 TC flag */
		buffer_in[BUFFER_SIZE - 1] = 1; /* Set unred packet flag */
	}
}

void UART_Init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN; /* Enable USART1 clock */
	RCC->AHBENR |= RCC_AHBENR_DMA1EN; /* Enable DMA1 clock */
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; /* Enable SYSCFG clock */
	
	SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART1TX_DMA_RMP; /* Remap USART1 TX DMA to channel 4 */
	SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART1RX_DMA_RMP; /* Remap USART1 RX DMA to channel 5 */
	
	USART1->BRR = 278U; /* 115200 Baud rate on APB2 bus on 32MHz (32000000 / 115200) */
	USART1->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR; /* Enable DMA transmitter and DMA receiver */
	
	DMA1_Channel4->CCR = 0;
	DMA1_Channel4->CPAR = (uint32_t) (&(USART1->TDR)); /* (1) */
	DMA1_Channel4->CMAR = (uint32_t) (buffer_out); /* (2) */
	DMA1_Channel4->CCR |= DMA_CCR_MINC | DMA_CCR_DIR; /* (4) */
	DMA1_Channel4->CCR |= DMA_CCR_TCIE; /* Enable transfer complete interrupt */
	
	DMA1_Channel5->CCR = 0;
	DMA1_Channel5->CPAR = (uint32_t) (&(USART1->RDR)); /* (1) */
	DMA1_Channel5->CMAR = (uint32_t) (buffer_in); /* (2) */
	DMA1_Channel5->CNDTR = sizeof(buffer_in) / sizeof(uint8_t);
	DMA1_Channel5->CCR |= DMA_CCR_CIRC | DMA_CCR_MINC; /* (4) */
	DMA1_Channel5->CCR |= DMA_CCR_HTIE | DMA_CCR_TCIE; /* Enable transfer complete interrupt */
	DMA1_Channel5->CCR |= DMA_CCR_EN; /* (8) */
	
	USART1->CR1 |= USART_CR1_RTOIE; /* Receiver timeout interrupt enable */
	USART1->CR2 |= USART_CR2_RTOEN; /* Receiver timeout enable */
	USART1->RTOR = 100000; /* Receiver timeout */
	
	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE; /* Enable TX RX UART */
	
	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);
}

uint8_t UART_RXNE(void)
{
	if (buffer_in_pos >= BUFFER_SIZE) buffer_in_pos = 0;
	return buffer_in[buffer_in_pos + PACKET_SIZE - 1]; /* Check last byte */
}

void UART_ReadBuffer(uint8_t *bytes)
{
	if (buffer_in_pos >= BUFFER_SIZE) buffer_in_pos = 0;
	for (uint8_t i = 0; i < PACKET_SIZE; i++) bytes[i] = buffer_in[buffer_in_pos + i];
	buffer_in[buffer_in_pos + PACKET_SIZE - 1] = 0; /* Clear last byte to indicate a red packet */
	buffer_in_pos += PACKET_SIZE;
}

uint8_t UART_ReadBytes(uint8_t *bytes, uint8_t size)
{
	NVIC_EnableIRQ(USART1_IRQn);
	for (uint8_t i = 0; i < size; i++)
	{
		bytes[i] = (uint8_t) USART1->RDR;
		while (((USART1->ISR & USART_ISR_RXNE) != USART_ISR_RXNE) && !time_out);
	}
	
	NVIC_DisableIRQ(USART1_IRQn);
	if (time_out)
	{
		time_out = 0;
		return 0;
	}
	return 1;
}

void UART_WriteBytes(const uint8_t *bytes, uint8_t size)
{
	for (uint8_t i = 0; i < size; i++) buffer_out[i] = bytes[i];
	
	while (tx_busy);
	//while ((USART1->ISR & USART_ISR_TC) != USART_ISR_TC);
	
	tx_busy = 1;
	DMA1_Channel4->CNDTR = size;
	DMA1_Channel4->CCR |= DMA_CCR_EN; /* (8) */
	
	//while ((DMA1->ISR & DMA_ISR_TCIF4) != DMA_ISR_TCIF4); /* Wait until DMA transfer is complete */
	//DMA1->IFCR |= DMA_IFCR_CHTIF4 /* Clear channel 4 interrupt flag */;
	//DMA1_Channel4->CCR &= ~DMA_CCR_EN; /* Disable DMA1 channel 4*/
	
	//for (uint8_t i = 0; i < size; i++)
	//{
	//	USART1->TDR = *bytes++;
	//	while ((USART1->ISR & USART_ISR_TC) != USART_ISR_TC);
	//}
}
