/*
 * mpu6050.h
 *
 *  Created on: Sep 22, 2023
 *      Author: fewletter
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32f4xx_hal.h"
#include "main.h"

#define WHO_AM_I       0x75
#define DEVICE_ADDRESS 0x68
#define PWR_MGMT_1     0x6B   /*Power Management 1*/
#define SMPRT_DIV      0x19   /*Sample Rate Divider*/
#define GYRO_CONFIG    0x1B   /*Gyroscope Configuration*/
#define ACCEL_CONFIG   0x1C   /*Gyroscope Configuration*/

#define ACCEL_X        0x3B   /*High bit of x accelerate data*/
#define ACCEL_Y        0x3D   /*High bit of y accelerate data*/
#define ACCEL_Z        0x3F   /*High bit of z accelerate data*/

#define GYRO_X         0x43   /*High bit of x gyro data*/
#define GYRO_Y         0x45   /*High bit of y gyro data*/
#define GYRO_Z         0x47   /*High bit of z gyro data*/

void mpu6050_Init(I2C_HandleTypeDef *hi2c);   /*Initial and check mpu6050*/
void mpu6050_Read_Accel(I2C_HandleTypeDef *hi2c);  /*Read the data through i2c*/
void mpu6050_Read_Gyro(I2C_HandleTypeDef *hi2c);   /*Read the data through i2c*/

#endif /* INC_MPU6050_H_ */
