#ifndef I2C_H
#define I2C_H

#include "stm32f030x6.h"

void I2C_Init(void);
void I2C_Transfer(uint8_t *bytes, uint32_t number_of_bytes, uint32_t address);
void I2C_Read(uint32_t reg, uint32_t address, uint8_t *byte);

#endif // I2C_H
