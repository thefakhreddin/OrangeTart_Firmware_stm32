#include "GPIO.h"
#include "config.h"

void GPIO_Init(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; /* Enable GPIOA clock */
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; /* Enable GPIOB clock */
	
	BT_STATE_GPIO_Port->MODER &= ~(0x03U << (BT_STATE_Pin << 1)); // Input mode
	BT_STATE_GPIO_Port->PUPDR |= 0x02U << (BT_STATE_Pin << 1); // Pull-down
	
	BATTERY_GPIO_Port->MODER &= ~(0x03U << (BATTERY_Pin << 1)); // Input mode
	BATTERY_GPIO_Port->PUPDR |= 0x02U << (BATTERY_Pin << 1); // Pull-down
	
	POWER_CONTROL_GPIO_Port->MODER |= 0x01U << (POWER_CONTROL_Pin << 1); // Output mode
	
	RGB_DATA_GPIO_Port->MODER |= 0x02U << (RGB_DATA_Pin << 1); /* Alternate function mode */
	RGB_DATA_GPIO_Port->AFR[0] |= 5U << (RGB_DATA_Pin << 2); /* Alternate function (TIM16_CH1 AF5) */
	RGB_DATA_GPIO_Port->OSPEEDR |= 0b11 << (RGB_DATA_Pin << 1); /* output high-speed */
	
	MOTOR_1_IN1_GPIO_Port->MODER |= 1U << (MOTOR_1_IN1_Pin << 1); // Output mode
	
	MOTOR_1_IN2_GPIO_Port->MODER |= 1U << (MOTOR_1_IN2_Pin << 1); // Output mode
	
	MOTOR_2_IN1_GPIO_Port->MODER |= 1U << (MOTOR_2_IN1_Pin << 1); // Output mode
	
	MOTOR_2_IN2_GPIO_Port->MODER |= 1U << (MOTOR_2_IN2_Pin << 1); // Output mode
	
	MOTOR_1_PWM_GPIO_Port->MODER |= 0x02U << (MOTOR_1_PWM_Pin << 1); // Alternate function mode
	MOTOR_1_PWM_GPIO_Port->AFR[1] |= 2U << ((MOTOR_1_PWM_Pin - 8) << 2); /* Alternate function (TIM1_CH1 AF2) */
	
	MOTOR_2_PWM_GPIO_Port->MODER |= 0x02U << (MOTOR_2_PWM_Pin << 1); // Alternate function mode
	MOTOR_2_PWM_GPIO_Port->AFR[1] |= 2U << ((MOTOR_2_PWM_Pin - 8) << 2); /* Alternate function (TIM1_CH4 AF2) */
	
	MOTOR_3_IN1_GPIO_Port->MODER |= 0x02U << (MOTOR_3_IN1_Pin << 1); // Alternate function mode
	MOTOR_3_IN1_GPIO_Port->AFR[0] |= 1U << (MOTOR_3_IN1_Pin << 2); /* Alternate function (TIM3_CH3 AF1) */
	
	MOTOR_3_IN2_GPIO_Port->MODER |= 0x02U << (MOTOR_3_IN2_Pin << 1); // Alternate function mode
	MOTOR_3_IN2_GPIO_Port->AFR[0] |= 1U << (MOTOR_3_IN2_Pin << 2); /* Alternate function (TIM3_CH4 AF1) */
	
	BUZZER_GPIO_Port->MODER |= 0x02U << (BUZZER_Pin << 1); // Alternate function mode
	BUZZER_GPIO_Port->AFR[0] |= 0x04U << (BUZZER_Pin << 2); /* Alternate function (TIM14_CH1 AF4) */
	
	BT_TX_GPIO_Port->MODER |= 0b10U << (BT_TX_Pin << 1); // Alternate function mode
	BT_TX_GPIO_Port->AFR[0] |= 1U << (BT_TX_Pin << 2); /* Alternate function */
	
	BT_RX_GPIO_Port->MODER |= 0b10U << (BT_RX_Pin << 1); // Alternate function mode
	BT_RX_GPIO_Port->AFR[0] |= 1U << (BT_RX_Pin << 2); /* Alternate function */
	
	I2C1_SCL_GPIO_Port->MODER |= 0b10U << (I2C1_SCL_Pin << 1); /* Alternate function mode */
	I2C1_SCL_GPIO_Port->OTYPER |= 1U << I2C1_SCL_Pin; /* output open-drain */
	I2C1_SCL_GPIO_Port->OSPEEDR |= 0b11 << (I2C1_SCL_Pin << 1); /* output high-speed */
	I2C1_SCL_GPIO_Port->AFR[0] |= 1U << (I2C1_SCL_Pin << 2); /* Alternate function (I2C1_SCL AF1) */
	
	I2C1_SDA_GPIO_Port->MODER |= 0b10U << (I2C1_SDA_Pin << 1); /* Alternate function mode */
	I2C1_SDA_GPIO_Port->OTYPER |= 1U << I2C1_SDA_Pin; /* output open-drain */
	I2C1_SDA_GPIO_Port->OSPEEDR |= 0b11 << (I2C1_SDA_Pin << 1); /* output high-speed */
	I2C1_SDA_GPIO_Port->AFR[0] |= 1U << (I2C1_SDA_Pin << 2); /* Alternate function (I2C1_SDA AF1) */
}

void GPIO_WritePin(GPIO_TypeDef *Port, uint32_t Pin, uint8_t PinState)
{
	if (PinState == 0)
	{
		Port->BSRR = 1 << (Pin + 16);
	}
	else
	{
		Port->BSRR = 1 << Pin;
	}
}

void GPIO_TogglePin(GPIO_TypeDef *Port, uint32_t Pin)
{
	Port->ODR ^= 1 << Pin;
}

uint8_t GPIO_ReadPin(GPIO_TypeDef *Port, uint32_t Pin)
{
	return (Port->IDR >> Pin) & 1;
}
