#include "ADC.h"

void ADC_Init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; /* Enable ADC1 clock */
	
	RCC->CR2 |= RCC_CR2_HSI14ON; /* Turn HSI14 on */
	while ((RCC->CR2 & RCC_CR2_HSI14RDY) != RCC_CR2_HSI14RDY); /* Wait for HSI14 to be ready */
	
	ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE_Msk; /* Select HSI14 as ADC1 clock source */
	
	if ((ADC1->CR & ADC_CR_ADEN) == ADC_CR_ADEN) /* Check if ADC1 is enabled */
	{
		ADC1->CR |= ADC_CR_ADDIS; /* Disable ADC1 */
		while ((ADC1->CR & ADC_CR_ADEN) == ADC_CR_ADEN); /* Wait for ADC1 to be disabled */
	}
	
	ADC1->CR |= ADC_CR_ADCAL; /* Calibrate ADC1 */
	while ((ADC1->CR & ADC_CR_ADCAL) == ADC_CR_ADCAL); /* Wait for ADC1 to be calibrated */
	
	ADC1->CHSELR |= ADC_CHSELR_CHSEL1; /* Select ADC1 channel 1 */
	
	ADC1->SMPR |= ADC_SMPR1_SMPR_2; /* 41.5 ADC clock cycles sampling time */
	
	ADC1->CR |= ADC_CR_ADEN; /* Enable ADC1 */
	while ((ADC1->ISR & ADC_ISR_ADRDY) != ADC_ISR_ADRDY); /* Wait for ADC1 to be ready */
}

uint16_t ADC_Read(void)
{
	ADC1->CR |= ADC_CR_ADSTART; /* Start conversion */
	while ((ADC1->ISR & ADC_ISR_EOC) != ADC_ISR_EOC); /* Wait for the end of conversion */
	
	return (uint16_t) ADC1->DR;
}
