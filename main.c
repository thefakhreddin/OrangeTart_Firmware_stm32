#include "main.h"
#include "config.h"


#define BATTERY_CHECK_INTERVAL 100
#define BT_STATUS_CHECK_INTERVAL 100
	
static uint8_t connected = 0;

static void OnStartup(void);
static void OnConnect(void);
static void OnDisconnect(void);
static void OnShutdown(void);
static void BatteryCheck(void);

int main(void)
{
	FLASH_Init();
	RCC_Init();
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock / 1000);
	NVIC_SetPriority(SysTick_IRQn, 14);
	GPIO_Init();
	UART_Init();
	I2C_Init();
	ADC_Init();
	Motor_Init();
	WS2812B_Init();
	Buzzer_Init();
	Motor_Drive(MOTOR_ALL, MOTOR_CW, 0);
	
	GPIO_WritePin(POWER_CONTROL_GPIO_Port, POWER_CONTROL_Pin, 1);
	
	TIM_Delay(500);
	MPU6050_Init();
	
	OnStartup();
	
	//print("\r\nAT\r\n");
	//uint16_t v = ADC_Read();
	//print("voltage: %d\n", v);
	
	uint32_t last_battery_check = ms_ticks;
	uint32_t last_bt_status_check = ms_ticks;
	
	while (1)
	{
		if (ms_ticks - last_battery_check > BATTERY_CHECK_INTERVAL)
		{
			BatteryCheck();
			last_battery_check = ms_ticks;
		}
		
		if (ms_ticks - last_bt_status_check > BT_STATUS_CHECK_INTERVAL)
		{
			if (GPIO_ReadPin(BT_STATE_GPIO_Port, BT_STATE_Pin) && !connected) OnConnect();
			else if (!GPIO_ReadPin(BT_STATE_GPIO_Port, BT_STATE_Pin) && connected) OnDisconnect();
			last_bt_status_check = ms_ticks;
		}
		
		if (UART_RXNE())
		{
			uint8_t com[PACKET_SIZE] = { 0 };
			UART_ReadBuffer(com);
			Data_Handler_Handle(com);
		}
		
		if (is_executing && g_closed_loop_motion == GYRO_MODE_OFF)
		{
			if (ms_ticks - g_wait_start >= g_wait_duration)
			{
				g_wait_start = 0;
				g_wait_duration = 0;

				if (++g_executing_index >= g_program_len)
				{
#if DEBUG_MODE
					Serial.println("End of program");
#endif
					/* End of program */
					is_executing = 0;
					//Data_Handler_Stop();
					uint8_t stop_bytes[2] = {0x0b, 0};
					UART_WriteBytes(stop_bytes, 2);
					//TIM_Delay(10);
					//uint8_t clear_bytes[1] = {0};
					//UART_WriteBytes(clear_bytes, 1);
#if DEBUG_MODE
					Serial.println("End of program confirmed");
#endif
				}
				else
				{
					/* Execute next command */
					Execute(g_executing_index);
				}
			}
		}
  }
}

void OnStartup(void)
{
	WS_QueueSimple(LED_1, ORANGE);
	WS_QueueSimple(LED_6, ORANGE);
	WS_Send();
	TIM_Delay(100);
	WS_QueueSimple(LED_2, ORANGE);
	WS_QueueSimple(LED_5, ORANGE);
	WS_Send();
	TIM_Delay(100);
	WS_QueueSimple(LED_3, ORANGE);
	WS_QueueSimple(LED_4, ORANGE);
	WS_Send();
	
	Buzzer_Tone(NOTE_C6, 175);
	TIM_Delay(350 * .75 - 175);
	Buzzer_Tone(NOTE_D6, 100);
	TIM_Delay(175 * .75 - 100);
	Buzzer_Tone(NOTE_E6, 325);
	TIM_Delay(175 * .75 - 325);
	Buzzer_Tone(NOTE_A6, 350);
	Buzzer_Tone(NOTE_G6, 350);
	
	WS_QueueSimple(LED_1, OFF);
	WS_QueueSimple(LED_6, OFF);
	WS_Send();
	TIM_Delay(100);
	WS_QueueSimple(LED_2, OFF);
	WS_QueueSimple(LED_5, OFF);
	WS_Send();
	TIM_Delay(100);
	WS_QueueSimple(LED_3, OFF);
	WS_QueueSimple(LED_4, OFF);
	WS_Send();
}

void OnConnect(void)
{
	WS_QueueSimple(LED_ALL, BLUE);
	WS_Send();
	TIM_Delay(100);
	WS_QueueSimple(LED_ALL, OFF);
	WS_Send();
	TIM_Delay(100);
	WS_QueueSimple(LED_ALL, BLUE);
	WS_Send();
	TIM_Delay(200);
	WS_QueueSimple(LED_ALL, OFF);
	WS_Send();
	/*
	Motor_Drive(MOTOR_1, MOTOR_CCW, 100);
	Motor_Drive(MOTOR_2, MOTOR_CCW, 100);
	TIM_Delay(500);
	Motor_Drive(MOTOR_1, MOTOR_CW, 100);
	Motor_Drive(MOTOR_2, MOTOR_CW, 100);
	TIM_Delay(800);
	Motor_Drive(MOTOR_1, MOTOR_CCW, 100);
	Motor_Drive(MOTOR_2, MOTOR_CCW, 100);
	TIM_Delay(500);
	Motor_Drive(MOTOR_ALL, MOTOR_CW, 0);
	*/
	connected = 1;
}

void OnDisconnect(void)
{
	Data_Handler_Stop();
	WS_QueueSimple(LED_ALL, RED);
	WS_Send();
	TIM_Delay(150);
	WS_QueueSimple(LED_ALL, OFF);
	WS_Send();
	TIM_Delay(150);
	WS_QueueSimple(LED_ALL, RED);
	WS_Send();
	TIM_Delay(300);
	WS_QueueSimple(LED_ALL, OFF);
	WS_Send();
	
	connected = 0;
}

void OnShutdown(void)
{
	WS_QueueSimple(LED_ALL, OFF);
	WS_Send();
	Motor_Drive(MOTOR_ALL, MOTOR_CW, 0);
	GPIO_WritePin(POWER_CONTROL_GPIO_Port, POWER_CONTROL_Pin, 0);
}

void BatteryCheck(void)
{
  uint16_t battery_analog_read = ADC_Read();
	if (battery_analog_read < 2234) OnShutdown();
}
