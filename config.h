#ifndef CONFIG_H
#define CONFIG_H

#define BATTERY_Pin 						1U
#define BATTERY_GPIO_Port 			GPIOA

//#define BUZZER_Pin 							4U
//#define BUZZER_GPIO_Port 				GPIOA

#define BUZZER_Pin 							7U
#define BUZZER_GPIO_Port 				GPIOA

//#define BT_STATE_Pin 						3U
//#define BT_STATE_GPIO_Port 			GPIOB

#define BT_STATE_Pin 						4U
#define BT_STATE_GPIO_Port 			GPIOA

#define POWER_CONTROL_Pin 			4U
#define POWER_CONTROL_GPIO_Port GPIOB

#define BT_TX_Pin 							2U
#define BT_TX_GPIO_Port 				GPIOA

#define BT_RX_Pin 							3U
#define BT_RX_GPIO_Port 				GPIOA

#define RGB_DATA_Pin 						6U
#define RGB_DATA_GPIO_Port 			GPIOA

#define MOTOR_1_PWM_Pin 				8U
#define MOTOR_1_PWM_GPIO_Port 	GPIOA

#define MOTOR_2_PWM_Pin 				11U
#define MOTOR_2_PWM_GPIO_Port 	GPIOA

#define MOTOR_1_IN2_Pin 				10U
#define MOTOR_1_IN2_GPIO_Port 	GPIOA

#define MOTOR_1_IN1_Pin 				9U
#define MOTOR_1_IN1_GPIO_Port 	GPIOA

#define MOTOR_2_IN1_Pin 				15U
#define MOTOR_2_IN1_GPIO_Port 	GPIOA

#define MOTOR_2_IN2_Pin 				12U
#define MOTOR_2_IN2_GPIO_Port 	GPIOA

#define MOTOR_3_IN1_Pin 				0U
#define MOTOR_3_IN1_GPIO_Port 	GPIOB

#define MOTOR_3_IN2_Pin 				1U
#define MOTOR_3_IN2_GPIO_Port 	GPIOB

#define I2C1_SCL_Pin 						6U
#define I2C1_SCL_GPIO_Port 			GPIOB

#define I2C1_SDA_Pin 						7U
#define I2C1_SDA_GPIO_Port 			GPIOB




#define BUZZER_TIM 							TIM14
#define BUZZER_TIM_CHANNEL 			CCR1

#define WS_TIM 									TIM16
#define WS_TIM_CHANNEL 					CCR1

#define MOTOR_1_TIM 						TIM1
#define MOTOR_1_TIM_CHANNEL 		CCR1

#define MOTOR_1_TIM 						TIM1
#define MOTOR_1_TIM_CHANNEL 		CCR1

#define MOTOR_2_TIM 						TIM1
#define MOTOR_2_TIM_CHANNEL 		CCR4

#define MOTOR_3_TIM 						TIM3
#define MOTOR_3_IN1_TIM_CHANNEL CCR3
#define MOTOR_3_IN2_TIM_CHANNEL CCR4

#endif // CONFIG_H
