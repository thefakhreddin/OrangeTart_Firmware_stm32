#include "I2C.h"
#include "config.h"
#include "WS2812B.h"
#include "Tim.h"
#include "serial_debug.h"


static void I2C_SendStart(void);
static void I2C_SendStop(void);

/*void I2C1_IRQHandler(void)
{
	if ((I2C1->ISR & I2C_ISR_BERR) == I2C_ISR_BERR)
	{
		I2C1->ICR |= I2C_ICR_BERRCF;
		WS_QueueSimple(LED_1, BLUE);
		WS_Send();
		TIM_Delay(2000);
		WS_QueueSimple(LED_1, OFF);
		WS_Send();
	}
	else
	{
		WS_QueueSimple(LED_1, GREEN);
		WS_Send();
		TIM_Delay(2000);
		WS_QueueSimple(LED_1, OFF);
		WS_Send();
	}
}*/

void I2C_Init(void)
{	
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; /* Enable I2C1 clock */
	I2C1->TIMINGR = (uint32_t) 0x2000090E; /* Using CubeMX I2C timing calculator */
	//I2C1->CR1 |= I2C_CR1_ERRIE | I2C_CR1_NACKIE; /* Enable I2C1 */
	I2C1->CR1 |= I2C_CR1_PE; /* Enable error interrupt */
	
	//NVIC_EnableIRQ(I2C1_IRQn);
	//NVIC_SetPriority(I2C1_IRQn, 15);
}

void I2C_SendStart(void)
{
	I2C1->CR2 |= I2C_CR2_START;
	while ((I2C1->CR2 & I2C_CR2_START) != I2C_CR2_START);
}

void I2C_SendStop(void)
{
	I2C1->CR2 |= I2C_CR2_STOP;
	while ((I2C1->CR2 & I2C_CR2_STOP) != I2C_CR2_STOP);
}

void I2C_Transfer(uint8_t *bytes, uint32_t number_of_bytes, uint32_t address)
{
	I2C1->CR2 &= ~I2C_CR2_NBYTES; // Clear nbytes
	I2C1->CR2 |= number_of_bytes << I2C_CR2_NBYTES_Pos; // nbytes to transfer
	I2C1->CR2 |= address << 1; // Slave address
	I2C1->CR2 &= ~I2C_CR2_AUTOEND; // Disable auto end
	I2C1->CR2 &= ~I2C_CR2_RD_WRN; // Write Mode
	
	I2C_SendStart();
	
	for (uint16_t i = 0; i < number_of_bytes; i++)
	{
		while ((I2C1->ISR & I2C_ISR_TXE) != I2C_ISR_TXE);
		I2C1->TXDR = bytes[i];
	}
	while ((I2C1->ISR & I2C_ISR_TC) != I2C_ISR_TC);
	
	I2C_SendStop();
}

void I2C_Read(uint32_t reg, uint32_t address, uint8_t *byte)
{
	// ---------------- write
	I2C1->CR2 &= ~I2C_CR2_NBYTES; // Clear nbytes
	I2C1->CR2 |= 1 << I2C_CR2_NBYTES_Pos; // nbytes to transfer
	I2C1->CR2 |= address << 1; // Slave address
	I2C1->CR2 &= ~I2C_CR2_AUTOEND; // Disable auto end
	I2C1->CR2 &= ~I2C_CR2_RD_WRN; // Write Mode
	
	I2C_SendStart();
	
	while ((I2C1->ISR & I2C_ISR_TXE) != I2C_ISR_TXE);
	I2C1->TXDR = reg;
	
	while ((I2C1->ISR & I2C_ISR_TC) != I2C_ISR_TC);
	
	// ---------------- read
	I2C1->CR2 &= ~I2C_CR2_NBYTES; // Clear nbytes
	I2C1->CR2 |= 1 << I2C_CR2_NBYTES_Pos; // nbytes to read
	I2C1->CR2 |= I2C_CR2_RD_WRN; // Read Mode
	//I2C1->CR2 |= I2C_CR2_AUTOEND; // Enable auto end
	
	I2C_SendStart();
	
	while ((I2C1->ISR & I2C_ISR_RXNE) != I2C_ISR_RXNE);
	*byte = I2C1->RXDR;
	
	while ((I2C1->ISR & I2C_ISR_TC) != I2C_ISR_TC);
	
	I2C_SendStop();
}
