#include "data_handler.h"
#include "config.h"
#include "TIM.h"
#include "USART.h"
#include "motor.h"
#include "config.h"
#include "MPU6050.h"
#include "WS2812B.h"
#include "buzzer.h"
#include "ADC.h"
#include "stdlib.h"


/* Battery */
#define MAX_BATTERY_VOLTAGE 2.1 									// Maximum allowed battery voltage
#define MIN_BATTERY_VOLTAGE 1.85									// Minimum allowed battery voltage
#define ANALOG_READ_GAIN 0.5                     // Voltage divider gain (100K/(100K+100K) = 0.404 is correct)
#define ARDUINO_ANALOG_READ_CONST 0.000805664         // Arduino analog read volt per ADC value
#define MAX_BATTERY_A_READ (MAX_BATTERY_VOLTAGE * ANALOG_READ_GAIN) / ARDUINO_ANALOG_READ_CONST      // Maximum allowed battery voltage (AD converted)
#define MIN_BATTERY_A_READ (MIN_BATTERY_VOLTAGE * ANALOG_READ_GAIN) / ARDUINO_ANALOG_READ_CONST      // Minimum allowed battery voltage (AD converted)

#define PROGRAMMING_TIMEOUT 1000


/* Protocol IDs */
#define MOTOR_DRIVE         0x01
#define DIFFERENTIAL_DRIVE  0x02
#define LINEAR_MOVEMENT     0x03
#define CIRCULAR_MOVEMENT   0x04
#define ON_BOARD_LED        0x05
#define BUZZER              0x06
#define ACTION              0x07
#define GYRO_LINEAR_DRIVE   0x08
#define GYRO_CIRCULAR_DRIVE 0x09
#define PROGRAM             0x0a
#define EXECUTE             0x0b
#define WAIT                0x0c
#define STOP                0x0d
#define LOOP_START 			    0x0e
#define LOOP_END 			      0x0f
#define LOOP_INF_END        0x10
#define BATTERY_INFO        0x11
#define ERROR               0xff


/* Error IDs */
#define ERROR_PROGRAMMING_TIMEOUT   0x01
#define ERROR_NOT_ENOUGH_MEMORY     0x02
#define ERROR_UNKNOWN_COMMAND       0x03


static const uint8_t CMD_ARG_NUM[] = { 0, 4, 4, 3, 3, 3, 5, 2, 5, 3, 2, 1, 3, 1, 1, 2, 1 };


//static const uint8_t start_bytes[2] = {EXECUTE, 1};
static const uint8_t stop_bytes[2] = {EXECUTE, 0};


/* Programming */
//GyroMovementMode closedLoopMotionMode = GYRO_MODE_OFF;
uint8_t g_program_len;
uint8_t **g_commands = NULL;
uint8_t is_executing = 0;
uint8_t g_executing_index = 0;
uint32_t g_wait_start = 0;
uint32_t g_wait_duration = 0;


uint32_t p1 = 0;
uint32_t p2 = 0;

static void FreeProgram(uint16_t length);
static void BTSendBatteryInfo(void);
static uint32_t map(uint32_t num, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max);
void gyro_block_check(void);

void Data_Handler_Handle(uint8_t *data)
{
  switch (data[0])
  {
    case MOTOR_DRIVE:
      {
#if DEBUG_MODE
        Serial.println("** MOTOR_DRIVE **");
#endif
        MotorIndex motor_index = (MotorIndex) data[1];
        MotorDirection motor_dir = (MotorDirection) data[2];
        uint8_t motor_speed = data[3];

#if DEBUG_MODE
        Serial.print("<");
        Serial.print(motor_index);
        Serial.print(">");
        Serial.print("<");
        Serial.print(motor_dir);
        Serial.print(">");
        Serial.print("<");
        Serial.print(motor_speed);
        Serial.println(">");
#endif

        Motor_Drive(motor_index, motor_dir, motor_speed);

        if (!motor_speed)
        {
          /* Disable gyroscope aided movements */
          g_closed_loop_motion = GYRO_MODE_OFF;
        }
      }
      break;

    case DIFFERENTIAL_DRIVE:
      {
#if DEBUG_MODE
        Serial.println("** DIFFERENTIAL_DRIVE **");
#endif
        MotorDirection motor_left_dir = (MotorDirection) data[1];
        uint8_t motor_left_speed = data[2];
        MotorDirection motor_right_dir = (MotorDirection) data[3];
        uint8_t motor_right_speed = data[4];
				
				if (!motor_left_speed && !motor_right_speed)
				{
					int a = 0;
				}

#if DEBUG_MODE
        Serial.print("<");
        Serial.print(motor_left_dir);
        Serial.print(">");
        Serial.print("<");
        Serial.print(motor_left_speed);
        Serial.print(">");
        Serial.print("<");
        Serial.print(motor_right_dir);
        Serial.print(">");
        Serial.print("<");
        Serial.print(motor_right_speed);
        Serial.println(">");
#endif

        Motor_Drive(MOTOR_1, motor_left_dir, motor_left_speed);
        Motor_Drive(MOTOR_2, motor_right_dir, motor_right_speed);

        /* Disable gyroscope aided movements */
        g_closed_loop_motion = GYRO_MODE_OFF;
      }
      break;

    case LINEAR_MOVEMENT:
      {
#if DEBUG_MODE
        Serial.println("** LINEAR_MOVEMENT **");
#endif
        uint8_t forward = data[1];
        uint8_t motorSpeed = data[2];

        if (forward)
        {
          Motor_Drive(MOTOR_1, MOTOR_CCW, motorSpeed);
          Motor_Drive(MOTOR_2, MOTOR_CW, motorSpeed);
        }
        else
        {
          Motor_Drive(MOTOR_1, MOTOR_CW, motorSpeed);
          Motor_Drive(MOTOR_2, MOTOR_CCW, motorSpeed);
        }
      }
      break;

    case CIRCULAR_MOVEMENT:
      {
#if DEBUG_MODE
        Serial.println("** CIRCULAR_MOVEMENT **");
#endif
        uint8_t clockwise = data[1];
        uint8_t motorSpeed = data[2];

        if (clockwise)
        {
          Motor_Drive(MOTOR_1, MOTOR_CCW, motorSpeed);
          Motor_Drive(MOTOR_2, MOTOR_CCW, motorSpeed);
        }
        else
        {
          Motor_Drive(MOTOR_1, MOTOR_CW, motorSpeed);
          Motor_Drive(MOTOR_2, MOTOR_CW, motorSpeed);
        }
      }
      break;

    case ON_BOARD_LED:
      {
#if DEBUG_MODE
        Serial.println("** ON_BOARD_LED **");
#endif
        LEDIndex led_index = (LEDIndex) data[1];
        LEDColor color_index = (LEDColor) data[2];

        WS_QueueSimple(led_index , color_index);
				WS_Send();
      }
      break;

    case BUZZER:
      {
#if DEBUG_MODE
        Serial.println("** BUZZER **");
#endif
        uint16_t frequency = (data[1] << 8) | data[2];
        uint16_t duration = (data[3] << 8) | data[4];
				if (!frequency)
				{
					Buzzer_ToneStop();
				}
				else
				{					
					if (duration) Buzzer_Tone(frequency, duration);
					else Buzzer_ToneStart(frequency);
				}
      }
      break;

    case GYRO_LINEAR_DRIVE:
#if DEBUG_MODE
      Serial.println("** GYRO_LINEAR_DRIVE **");
#endif
      {
        uint8_t forward = data[1];
        uint8_t speed = data[2];
        uint16_t duration = data[3] << 8 | data [4];
				CNTRLR_StartForward(forward, speed, duration);
        /*g_yaw = 0;
        g_loading_speed = 0;
        last_gyro_read_time = micros();
        g_start_time = millis();

        closedLoopMotionMode = GYRO_MODE_LINEAR;

        if (g_forward)
        {
          Motor_Drive(MOTOR_1, MOTOR_CCW, g_speed);
          Motor_Drive(MOTOR_2, MOTOR_CW, g_speed);
        }
        else
        {
          Motor_Drive(MOTOR_1, MOTOR_CW, g_speed);
          Motor_Drive(MOTOR_2, MOTOR_CCW, g_speed);
        }*/
      }
      break;

    case GYRO_CIRCULAR_DRIVE:
#if DEBUG_MODE
      Serial.println("** GYRO_CIRCULAR_DRIVE **");
#endif
      {
				TIM_Delay(10);
        uint8_t clockwise = data[1];
				// TODO: Handle this cleaner.
        int16_t desired_angle = data[2];
				if (clockwise)
				{
					desired_angle *= -1;
				}
				CNTRLR_StartTurn(desired_angle);
      }
      break;

    case LOOP_START:
#if DEBUG_MODE
      Serial.println("** LOOP_START **");
#endif
      break;
		
		case LOOP_END:
#if DEBUG_MODE
      Serial.println("** LOOP_END **");
      Serial.print("Loop remaining: ");
      Serial.println(g_commands[g_executing_index][1]);
#endif
      {
        if (--g_commands[g_executing_index][1] > 0)
        {
          /* Repeat loop */
          uint8_t nested_loop_count = 0;
          while (g_commands[--g_executing_index][0] != LOOP_START && nested_loop_count == 0)
          {
            if (g_commands[g_executing_index][0] == LOOP_END)
            {
              nested_loop_count++;
            }
          }
        }
      }
      break;

    case LOOP_INF_END:
#if DEBUG_MODE
      Serial.println("** LOOP_INF_END **");
#endif
      {
        /* Repeat loop */
        uint8_t nested_loop_count = 0;
        while (g_commands[--g_executing_index][0] != LOOP_START && nested_loop_count == 0)
        {
          if (g_commands[g_executing_index][0] == LOOP_END)
          {
            nested_loop_count++;
          }
        }
      }
      break;


    case PROGRAM:
      {
#if DEBUG_MODE
        Serial.println("** PROGRAM **");
#endif
				p1 = ms_ticks;
        if (g_commands != NULL)
        {
          /* Uncompleted program already available */
          /* TODO: Decide whether accept or discard incoming program */
          FreeProgram(g_program_len);
          //return;
        }
        g_program_len = data[1];
#if DEBUG_MODE
        Serial.print("Length of program: ");
        Serial.println(g_program_len);
#endif
        if ((g_commands = (uint8_t**) malloc(g_program_len * sizeof(uint8_t*)))
            != NULL)
        {
          for (uint16_t i = 0; i < g_program_len; i++)
          {
            /* Wait until data is received */
            uint32_t t = ms_ticks;
            while (!UART_RXNE())
            {
              if (ms_ticks - t > PROGRAMMING_TIMEOUT)
              {
#if DEBUG_MODE
                Serial.println(
                  "Bluetooth connection time-out. Canceling Programming...");
#endif
                // No data received, cancel programming.
                /* Clear Program Data */
                FreeProgram(i);
								uint8_t bt_data_out[2] = {ERROR, ERROR_PROGRAMMING_TIMEOUT};
								UART_WriteBytes(bt_data_out, 2);
                return;
              }
            }

            uint8_t com[PACKET_SIZE] = { 0 };
						//UART_ReadBytes(com, PACKET_SIZE);
						UART_ReadBuffer(com);
						
						/*for (uint16_t i = 0; i < PACKET_SIZE; i++)
						{
							com[i] = buffer_in[i];
						}*/

            if (com[0] < sizeof(CMD_ARG_NUM))
            {
              /* ID valid */
              if ((g_commands[i] = (uint8_t*) malloc(
                                     CMD_ARG_NUM[com[0]] * sizeof(uint8_t))) != NULL)
              {
#if DEBUG_MODE
                Serial.print("Command ");
                Serial.print(i);
                Serial.print(": ");
#endif
                for (uint8_t j = 0; j < CMD_ARG_NUM[com[0]]; j++)
                {
                  g_commands[i][j] = com[j];
#if DEBUG_MODE
                  Serial.print("<");
                  Serial.print(g_commands[i][j]);
                  Serial.print(">");
#endif
                }
#if DEBUG_MODE
                Serial.println(".");
#endif

                //                if (j < CMD_ARG_NUM[commands[i][0]])
                //                {
                //#if DEBUG_MODE
                //                Serial.println("Command uncomplete.");
                //#endif
                //                }
              }
              else
              {
#if DEBUG_MODE
                Serial.println(
                  "Not enough programming space. Canceling Programming...");
#endif
                // Not enough programming space.
                /* Clear Program Data */
                FreeProgram(i);
								uint8_t bt_data_out[2] = {ERROR, ERROR_NOT_ENOUGH_MEMORY};
								UART_WriteBytes(bt_data_out, 2);
                return;
              }
            }
            else
            {
#if DEBUG_MODE
              Serial.println(
                "Undefined command ID. Canceling Programming...");
#endif
              // Not enough programming space.
              /* Clear Program Data */
              FreeProgram(i);
							uint8_t bt_data_out[2] = {ERROR, ERROR_UNKNOWN_COMMAND};
              UART_WriteBytes(bt_data_out, 2);
              return;
            }
          }
        }
        else
        {
#if DEBUG_MODE
          Serial.println(
            "Not enough programming space. Canceling Programming...");
#endif
          // Not enough programming space
					uint8_t bt_data_out[2] = {ERROR, ERROR_NOT_ENOUGH_MEMORY};
          UART_WriteBytes(bt_data_out, 2);
        }
      }
			p2 = ms_ticks;
			uint32_t dp = p2 - p1;
      break;

    case EXECUTE:
      {
#if DEBUG_MODE
        Serial.println("** EXECUTE **");
#endif
        is_executing = 1;
        /* MPU6050 Zero Calibration */
        //MPU6050_CalculateError();
        Execute(g_executing_index = 0);
      }
      break;

    case STOP:
      {
#if DEBUG_MODE
        Serial.println("** STOP **");
#endif
        is_executing = 0;
        Data_Handler_Stop();
      }
      break;

    case WAIT:
      {
#if DEBUG_MODE
        Serial.println("** WAIT **");
#endif
        g_wait_start = ms_ticks;
        g_wait_duration = g_commands[g_executing_index][1] << 8
                          | g_commands[g_executing_index][2];

#if DEBUG_MODE
        Serial.print("Delay requested for ");
        Serial.print(g_wait_duration);
        Serial.println(" ms.");
#endif
      }
      break;

    case ACTION:
      {
#if DEBUG_MODE
        Serial.println("** ACTION **");
#endif
      }
      break;
			
    case BATTERY_INFO:
      {
#if DEBUG_MODE
        Serial.println("** BATTERY_INFO **");
#endif
        BTSendBatteryInfo();
      }
      break;


    default:
#if DEBUG_MODE
      Serial.print("** UNDEFINED COMMAND --> **");
      Serial.println(data[0]);
#endif
      break;
  }
}

void Execute(uint16_t index)
{
	//UART_WriteBytes(start_bytes, 2);
  Data_Handler_Handle(g_commands[index]);
  //free(commands[index]);
}

void Data_Handler_Stop(void)
{
	CNTRLR_Stop();
  Motor_Drive(MOTOR_ALL, MOTOR_CW, 0); /* Stop Motors */
  WS_QueueSimple(LED_ALL, OFF); /* Stop LED */
	WS_Send();
	Buzzer_ToneStop(); /* Stop buzzer */

  /* Clear Program Data */
  FreeProgram(g_program_len);
	
	//UART_WriteBytes(stop_bytes, 2);
}

static void FreeProgram(uint16_t length)
{
  if (g_commands == NULL)
  {
#if DEBUG_MODE
    Serial.println("Already Free");
#endif
    return;
  }
  while (length--)
  {
    free(g_commands[length]);
  }
  free(g_commands);
  g_commands = NULL;
#if DEBUG_MODE
  Serial.println("Freed successfully");
#endif
}

void BTSendBatteryInfo(void)
{
  // Convert adc value to battery percentage
  uint16_t battery_analog_read = ADC_Read();
  uint8_t battery_percentage = map(battery_analog_read, 2234, 2482, 0, 100);
  uint8_t bt_data_out[2] = {BATTERY_INFO, battery_percentage};
  UART_WriteBytes(bt_data_out, 2);

#if DEBUG_MODE
  Serial.print("Battery percentage is ");
  Serial.println(battery_percentage);
#endif
}

uint32_t map(uint32_t num, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max)
{
	if (num <= in_min) return out_min;
	if (num >= in_max) return out_max;
	return (num - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
