#ifndef GPIO_H
#define GPIO_H

#include "stm32f030x6.h"

void GPIO_Init(void);
void GPIO_WritePin(GPIO_TypeDef *Port, uint32_t Pin, uint8_t PinState);
void GPIO_TogglePin(GPIO_TypeDef *Port, uint32_t Pin);
uint8_t GPIO_ReadPin(GPIO_TypeDef *Port, uint32_t Pin);

#endif // GPIO_H
