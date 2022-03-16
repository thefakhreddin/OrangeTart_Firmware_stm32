#include "MPU6050.h"
#include "serial_debug.h"
#include "I2C.h"


/* MPU6050 */
#define MPU6050_ADDR 		0x68
#define WHO_AM_I_REG 		0x75
#define PWR_MGMT_REG 		0x6B
#define SMPLRT_DIV_REG 	0x19
#define GYRO_CONFIG_REG 0x1B
#define GYRO_ZOUT_H_REG 0x47
#define GYRO_ZOUT_L_REG 0x48


static float g_yaw;
static float g_gyro_z_error;


void TIM16_IRQHandler(void)
{
	TIM16->SR &= ~TIM_SR_UIF;
	uint8_t gyro_z_msb, gyro_z_lsb;
	I2C_Read(GYRO_ZOUT_H_REG, MPU6050_ADDR, &gyro_z_msb);
	I2C_Read(GYRO_ZOUT_L_REG, MPU6050_ADDR, &gyro_z_lsb);
	int16_t gyro_z = gyro_z_msb << 8 | gyro_z_lsb;
	gyro_z = -(~gyro_z + 1);
	
	float gyro = (gyro_z / 131.0) - g_gyro_z_error;
	
	g_yaw += gyro * 0.01;
	
	#if DEBUG_MODE
	print("%f\r\n", g_yaw);
	#endif
}

void MPU6050_Init(void)
{
	uint8_t response;
	I2C_Read(WHO_AM_I_REG, MPU6050_ADDR, &response);
	
	#if DEBUG_MODE
	//print("response %d\r\n", response);
	#endif
	
	if (response == MPU6050_ADDR)
	{
		#if DEBUG_MODE
		print("MPU6050 OK.\r\n");
		#endif
		
		uint8_t data[2];
	
		data[0] = PWR_MGMT_REG;
		data[1] = 0x00;
		I2C_Transfer(data, sizeof(data), MPU6050_ADDR);
		
		data[0] = SMPLRT_DIV_REG;
		data[1] = 0x07;
		I2C_Transfer(data, sizeof(data), MPU6050_ADDR);
		
		data[0] = GYRO_CONFIG_REG;
		data[1] = 0x00;
		I2C_Transfer(data, sizeof(data), MPU6050_ADDR);
		
		#if DEBUG_MODE
		print("MPU6050 Setup Done.\r\n");
		#endif
	}
	else
	{
		#if DEBUG_MODE
		print("MPU6050 Error.\r\n");
		#endif
	}
}

float MPU6050_ReadRawZ(void)
{
	uint8_t gyro_z_msb, gyro_z_lsb;
	I2C_Read(GYRO_ZOUT_H_REG, MPU6050_ADDR, &gyro_z_msb);
	I2C_Read(GYRO_ZOUT_L_REG, MPU6050_ADDR, &gyro_z_lsb);
	int16_t gyro_z = gyro_z_msb << 8 | gyro_z_lsb;
	gyro_z = -(~gyro_z + 1);
	return gyro_z / 131.0;
}

float MPU6050_CalculateError(void)
{
	float gyro_z_error = 0;
	for (uint8_t i = 0; i < 200; i++)
	{
		gyro_z_error += MPU6050_ReadRawZ();
	}
	
	#if DEBUG_MODE
	print("error %f\r\n", gyro_z_error / 200.0);
	#endif
	return gyro_z_error / 200.0;
}
