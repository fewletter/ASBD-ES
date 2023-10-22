/*
 * mpu6050.c
 *
 *  Created on: Sep 22, 2023
 *      Author: fewletter
 */

#include "mpu6050.h"
#include <stdio.h>
#include <stdint.h>

void mpu6050_Init(I2C_HandleTypeDef *hi2c)
{
	uint8_t ret;
	HAL_I2C_Mem_Read(hi2c, (DEVICE_ADDRESS << 1), WHO_AM_I, 1, &ret, 1, 100);

	HAL_I2C_Mem_Write(hi2c, (DEVICE_ADDRESS << 1), PWR_MGMT_1, 1, 0, 1, 100);
	HAL_I2C_Mem_Write(hi2c, (DEVICE_ADDRESS << 1), SMPRT_DIV, 1, 0x07, 1, 100);
	HAL_I2C_Mem_Write(hi2c, (DEVICE_ADDRESS << 1), GYRO_CONFIG, 1, 0x00, 1, 100);
	HAL_I2C_Mem_Write(hi2c, (DEVICE_ADDRESS << 1), ACCEL_CONFIG , 1, 0x00, 1, 100);
	if (ret == 104)
		printf("MPU6050 is ready\n");
	else
		printf("MPU6050 isn't ready\n");


}

void mpu6050_Read_Accel(I2C_HandleTypeDef *hi2c)
{
	uint8_t data[6];
	uint8_t OK = HAL_I2C_Mem_Read(hi2c, (DEVICE_ADDRESS << 1), ACCEL_X, 1, data, 6, 100);

	printf("X axis acceleration: %d\n", ((int16_t)(data[0] << 8) | data[1]) / 16384.0);
	printf("Y axis acceleration: %d\n", ((int16_t)(data[2] << 8) | data[3]) / 16384.0);
	printf("Z axis acceleration: %d\n", ((int16_t)(data[4] << 8) | data[5]) / 16384.0);

}

void mpu6050_Read_Gyro(I2C_HandleTypeDef *hi2c)
{
	uint8_t data[6];
	uint8_t OK = HAL_I2C_Mem_Read(hi2c, (DEVICE_ADDRESS << 1), GYRO_X, 1, data, 6, 100);

	printf("X axis gyro: %d\n", ((int16_t)(data[0] << 8) | data[1]) / 131);
	printf("Y axis gyro: %d\n", ((int16_t)(data[2] << 8) | data[3]) / 131);
	printf("Z axis gyro: %d\n", ((int16_t)(data[4] << 8) | data[5]) / 131);
}
