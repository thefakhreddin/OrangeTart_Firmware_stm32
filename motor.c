#include "motor.h"
#include "config.h"
#include "GPIO.h"
#include "TIM.h"
#include "MPU6050.h"
#include "math.h"
#include "serial_debug.h"

#include "WS2812B.h"

#define SAMPLE_TIME 					5
#define BLOCK_CHECK_INTERVAL 	1000

#define CNTRLR_LINEAR_KP			5
#define CNTRLR_LINEAR_MAX 		160 // 80
#define CNTRLR_LINEAR_MIN 		10 // 5

#define CNTRLR_CIRCULAR_KP		5
#define CNTRLR_CIRCULAR_MAX 	120 // 60
#define CNTRLR_CIRCULAR_MIN 	80 // 45

GyroMovementMode g_closed_loop_motion = GYRO_MODE_OFF;

volatile static float g_yaw;
static float g_gyro_z_error;
static float g_desired_angle;
static uint8_t g_speed;
static uint8_t g_loading_speed;
static uint32_t g_duration;
static uint32_t g_start_time;
static uint8_t g_forward;
volatile static float g_block_check_last_yaw;
volatile uint32_t g_last_block_check;

static void CNTRLR_Turn(void);
static void CNTRLR_Forward(void);
static void CNTRLR_Block_Check(void);


void TIM17_IRQHandler(void)
{
	TIM17->SR &= ~TIM_SR_UIF;
	
	switch (g_closed_loop_motion)
	{
		case GYRO_MODE_LINEAR:
			g_yaw += (MPU6050_ReadRawZ() - g_gyro_z_error) * SAMPLE_TIME / 1000;
			if (g_yaw > 360) g_yaw -= 360;
			if (g_yaw < -360) g_yaw += 360;
			CNTRLR_Forward();
			break;
		
		case GYRO_MODE_CIRCULAR:
			g_yaw += (MPU6050_ReadRawZ() - g_gyro_z_error) * SAMPLE_TIME / 1000;
			if (g_yaw > 360) g_yaw -= 360;
			if (g_yaw < -360) g_yaw += 360;
			CNTRLR_Turn();
			break;
		
		case GYRO_MODE_OFF:
			break;
		
		default:
			break;
	}
}

void Motor_Init(void)
{
	// ##### TIM1 (Motor 1 & 2) #####
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; /* Enable TIM1 clock */
	
	/* 20KHz PWM Frequency */
	TIM1->PSC = 31;
	TIM1->ARR = 49;
	
	TIM1->CCR1 = 30;
	TIM1->CCR4 = 30;
	
	TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC4E; /* Capture/Compare 1 output enable */
	TIM1->CR1 |= TIM_CR1_ARPE; /* Auto-reload preload enable */
	TIM1->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
	TIM1->CCMR2 &= ~TIM_CCMR2_OC4M_Msk;
	TIM1->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE; /* PWM mode 1 and Preload enable */
	TIM1->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4PE; /* PWM mode 1 and Preload enable */
	TIM1->BDTR |= TIM_BDTR_MOE; /* Enable main output */
	TIM1->EGR |= TIM_EGR_UG; /* Update generation */
	TIM1->CR1 |= TIM_CR1_CEN; /* Enable timer */
	
	// ##### TIM1 (Motor 1 & 2) #####
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; /* Enable TIM3 clock */
	
	/* 1KHz PWM Frequency */
	TIM3->PSC = 31;
	TIM3->ARR = 999;
	
	TIM3->CCR2 = 400;
	TIM3->CCR3 = 499;
	TIM3->CCR4 = 800;
	
	TIM3->CCER |= TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E; /* Capture/Compare 1 output enable */
	TIM3->CR1 |= TIM_CR1_ARPE; /* Auto-reload preload enable */
	//TIM3->CCMR1 &= ~TIM_CCMR1_OC2M_Msk;
	TIM3->CCMR2 &= ~TIM_CCMR2_OC3M_Msk;
	TIM3->CCMR2 &= ~TIM_CCMR2_OC4M_Msk;
	//TIM3->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE; /* PWM mode 1 and Preload enable */
	TIM3->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE; /* PWM mode 1 and Preload enable */
	TIM3->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4PE; /* PWM mode 1 and Preload enable */
	TIM3->BDTR |= TIM_BDTR_MOE; /* Enable main output */
	TIM3->EGR |= TIM_EGR_UG; /* Update generation */
	TIM3->CR1 |= TIM_CR1_CEN; /* Enable timer */
	
	
	// ##### TIM17 (Controller) #####
	RCC->APB2ENR |= RCC_APB2ENR_TIM17EN; /* Enable TIM17 clock */
	
	TIM17->PSC = 31;
	TIM17->ARR = 4999;
	
	TIM17->CR1 |= TIM_CR1_ARPE; /* Auto-reload preload enable */
	TIM17->CCER &= ~TIM_CCER_CC1E; /* Capture/Compare 1 output disable */
	TIM17->DIER |= TIM_DIER_UIE; /* Enable update interrupt */
	TIM17->EGR |= TIM_EGR_UG; /* Update generation */
	
	NVIC_EnableIRQ(TIM17_IRQn);
	NVIC_SetPriority(TIM17_IRQn, 15);
}

void Motor_Drive(MotorIndex motor_index, MotorDirection dir, uint16_t pwm)
{
  if (motor_index == MOTOR_ALL)
  {
    Motor_Drive(MOTOR_1, MOTOR_CW, pwm);
    Motor_Drive(MOTOR_2, MOTOR_CW, pwm);
    Motor_Drive(MOTOR_3, MOTOR_CW, pwm);
    return;
  }

  if (motor_index == MOTOR_1)
  {
    if (dir == MOTOR_CW)
    {
			MOTOR_1_TIM->MOTOR_1_TIM_CHANNEL = pwm * MOTOR_1_TIM->ARR / UINT8_MAX;
      GPIO_WritePin(MOTOR_1_IN1_GPIO_Port, MOTOR_1_IN1_Pin, 0);
      GPIO_WritePin(MOTOR_1_IN2_GPIO_Port, MOTOR_1_IN2_Pin, 1);
    }
    if (dir == MOTOR_CCW)
    {
			MOTOR_1_TIM->MOTOR_1_TIM_CHANNEL = pwm * MOTOR_1_TIM->ARR / UINT8_MAX;
      GPIO_WritePin(MOTOR_1_IN1_GPIO_Port, MOTOR_1_IN1_Pin, 1);
      GPIO_WritePin(MOTOR_1_IN2_GPIO_Port, MOTOR_1_IN2_Pin, 0);
    }
  }

  if (motor_index == MOTOR_2)
  {
    if (dir == MOTOR_CW)
    {
			MOTOR_2_TIM->MOTOR_2_TIM_CHANNEL = pwm * MOTOR_2_TIM->ARR / UINT8_MAX;
      GPIO_WritePin(MOTOR_2_IN1_GPIO_Port, MOTOR_2_IN1_Pin, 0);
      GPIO_WritePin(MOTOR_2_IN2_GPIO_Port, MOTOR_2_IN2_Pin, 1);
    }
    if (dir == MOTOR_CCW)
    {
			MOTOR_2_TIM->MOTOR_2_TIM_CHANNEL = pwm * MOTOR_2_TIM->ARR / UINT8_MAX;
      GPIO_WritePin(MOTOR_2_IN1_GPIO_Port, MOTOR_2_IN1_Pin, 1);
      GPIO_WritePin(MOTOR_2_IN2_GPIO_Port, MOTOR_2_IN2_Pin, 0);
    }
  }

  if (motor_index == MOTOR_3)
  {
    if (!pwm)
    {
      /* Stop */
			MOTOR_3_TIM->CCR3 = 0;
			MOTOR_3_TIM->CCR4 = 0;
    }
    else
    {
      if (dir == MOTOR_CW)
      {
				MOTOR_3_TIM->CCR3 = pwm * MOTOR_3_TIM->ARR / UINT8_MAX;
				MOTOR_3_TIM->CCR4 = 0;
      }
      if (dir == MOTOR_CCW)
      {
				MOTOR_3_TIM->CCR3 = 0;
				MOTOR_3_TIM->CCR4 = pwm * MOTOR_3_TIM->ARR / UINT8_MAX;
      }
    }
  }
}

void CNTRLR_StartForward(uint8_t forward, uint8_t speed, uint32_t duration)
{
	g_yaw = 0;
	g_closed_loop_motion = GYRO_MODE_LINEAR;
	g_desired_angle = 0;
	g_forward = forward;
	g_speed = speed;
	g_duration = duration;
	g_start_time = ms_ticks;
	
	g_gyro_z_error = MPU6050_CalculateError();
	
	TIM17->CR1 |= TIM_CR1_CEN; /* Enable timer */
}

void CNTRLR_StartTurn(float desired_angle)
{
	g_yaw = 0;
	g_closed_loop_motion = GYRO_MODE_CIRCULAR;
	g_desired_angle = desired_angle;
	g_block_check_last_yaw = 0;
	g_last_block_check = ms_ticks;
	
	g_gyro_z_error = MPU6050_CalculateError();
	
	TIM17->CR1 |= TIM_CR1_CEN; /* Enable timer */
}

void CNTRLR_Stop(void)
{
	TIM17->CR1 &= ~TIM_CR1_CEN; /* Disable timer */
	TIM17->CNT = 0;
	g_closed_loop_motion = GYRO_MODE_OFF;
	Motor_Drive(MOTOR_ALL, MOTOR_CW, 0);
	TIM_Delay(50);
}

static void CNTRLR_Forward(void)
{
	if (ms_ticks - g_start_time > g_duration)
	{
		CNTRLR_Stop();
		return;
	}
  /*if (ms_ticks - g_start_time < g_time - (g_speed * SAMPLE_TIME) && ms_ticks - g_start_time < g_time / 2)
  {
    if (g_loading_speed < g_speed)
    {
      g_loading_speed += 2;
    }
  }

  if (ms_ticks - g_start_time > g_time - (g_speed * SAMPLE_TIME) && ms_ticks - g_start_time > g_time / 2)
  {
    if (g_loading_speed > 0)
    {
      g_loading_speed -= 2;
    }
  }

  if (ms_ticks - g_start_time >= g_time)
  {
    Motor_Drive(MOTOR_1, MOTOR_CCW, 0);
    Motor_Drive(MOTOR_2, MOTOR_CW, 0);
    g_closed_loop_motion = GYRO_MODE_OFF;
  }*/
	
	g_loading_speed = g_speed;

  float delta_t = -g_yaw;
  float cntrlr_output =  delta_t * CNTRLR_LINEAR_KP;
	cntrlr_output *= 255 / 360.0;
	
	if (cntrlr_output > 0)
	{
		if (cntrlr_output < CNTRLR_LINEAR_MIN)
		{
			cntrlr_output = CNTRLR_LINEAR_MIN;
		}
		if (cntrlr_output > CNTRLR_LINEAR_MAX)
		{
			cntrlr_output = CNTRLR_LINEAR_MAX;
		}
	}
	else
	{
		if (cntrlr_output > -CNTRLR_LINEAR_MIN)
		{
			cntrlr_output = -CNTRLR_LINEAR_MIN;
		}
		if (cntrlr_output < -CNTRLR_LINEAR_MAX)
		{
			cntrlr_output = -CNTRLR_LINEAR_MAX;
		}
	}

  if (g_forward)
  {
    if (cntrlr_output > 0)
    {
			Motor_Drive(MOTOR_1, MOTOR_CCW, g_loading_speed - cntrlr_output);
			Motor_Drive(MOTOR_2, MOTOR_CW, g_loading_speed + cntrlr_output);
    }
    else
    {
			Motor_Drive(MOTOR_1, MOTOR_CCW, g_loading_speed - cntrlr_output);
      Motor_Drive(MOTOR_2, MOTOR_CW, g_loading_speed - fabs(cntrlr_output));
    }
  }
  else
	{
    if (cntrlr_output > 0)
    {
      Motor_Drive(MOTOR_1, MOTOR_CW, g_loading_speed + cntrlr_output);
      Motor_Drive(MOTOR_2, MOTOR_CCW, g_loading_speed);
    }
    else
    {
      Motor_Drive(MOTOR_1, MOTOR_CW, g_loading_speed);
      Motor_Drive(MOTOR_2, MOTOR_CCW, g_loading_speed + fabs(cntrlr_output));
    }
  }
}

static void CNTRLR_Turn(void)
{
  float delta_t = g_desired_angle - g_yaw;
  float cntrlr_output =  delta_t * CNTRLR_CIRCULAR_KP;
	cntrlr_output *= 255 / 360.0;
	
	if (cntrlr_output > 0)
	{
		if (cntrlr_output < CNTRLR_CIRCULAR_MIN)
		{
			cntrlr_output = CNTRLR_CIRCULAR_MIN;
		}
		if (cntrlr_output > CNTRLR_CIRCULAR_MAX)
		{
			cntrlr_output = CNTRLR_CIRCULAR_MAX;
		}
    Motor_Drive(MOTOR_1, MOTOR_CW, cntrlr_output);
    Motor_Drive(MOTOR_2, MOTOR_CW, cntrlr_output);
	}
	else
	{
		if (cntrlr_output > -CNTRLR_CIRCULAR_MIN)
		{
			cntrlr_output = -CNTRLR_CIRCULAR_MIN;
		}
		if (cntrlr_output < -CNTRLR_CIRCULAR_MAX)
		{
			cntrlr_output = -CNTRLR_CIRCULAR_MAX;
		}
    Motor_Drive(MOTOR_1, MOTOR_CCW, fabs(cntrlr_output));
    Motor_Drive(MOTOR_2, MOTOR_CCW, fabs(cntrlr_output));
	}

  if (fabs(delta_t) <= .2)
  {
    CNTRLR_Stop();
  }
	
	if (ms_ticks - g_last_block_check > BLOCK_CHECK_INTERVAL)
	{
		CNTRLR_Block_Check();
		g_last_block_check = ms_ticks;
	}
}

void CNTRLR_Block_Check(void)
{
  if (fabs(g_yaw - g_block_check_last_yaw) < 5)
  {
    CNTRLR_Stop();
  }
  g_block_check_last_yaw = g_yaw;
}
