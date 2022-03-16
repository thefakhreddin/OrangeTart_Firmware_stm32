#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f030x6.h"

void MPU6050_Init(void);
float MPU6050_CalculateError(void);
float MPU6050_ReadRawZ(void);

#endif // MPU6050_H
