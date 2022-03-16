#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f030x6.h"

typedef enum
{
  MOTOR_ALL, MOTOR_1, MOTOR_2, MOTOR_3
} MotorIndex;

typedef enum
{
  MOTOR_CCW, MOTOR_CW
} MotorDirection;

typedef enum
{
  GYRO_MODE_OFF, GYRO_MODE_LINEAR, GYRO_MODE_CIRCULAR
} GyroMovementMode;


extern GyroMovementMode g_closed_loop_motion;

void Motor_Init(void);
void Motor_Drive(MotorIndex motor_index, MotorDirection dir, uint16_t pwm);
void CNTRLR_StartForward(uint8_t forward, uint8_t speed, uint32_t duration);
void CNTRLR_StartTurn(float desired_angle);
void CNTRLR_Stop(void);

#endif // MOTOR_H
