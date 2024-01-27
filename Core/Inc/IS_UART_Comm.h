/*
 * IS_UART_Comm.h
 *
 * Inertial Sense IMU NEMA
 *
 *  Created on: Dec 26, 2023
 *      Author: fewletter
 */

#ifndef INC_IS_UART_COMM_H_
#define INC_IS_UART_COMM_H_

#include <stdint.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include <stdbool.h>

#define UART_BUFFER_SIZE 145
#define PIMU  0x1
#define PINS1 (0x1 << 1)
#define container_of(ptr, type, member) ({                      \
		const typeof( ((type *)0)->member ) *__mptr = (ptr);    \
		(type *)( (char *)__mptr - offsetof(type,member) );})

typedef struct
{
  uint8_t buffer[UART_BUFFER_SIZE];
  volatile uint8_t* head;
  volatile uint8_t* tail;
  volatile uint8_t* breakpoint;
  uint8_t datatype;
} CommPacket;

typedef struct
{
	volatile float theta;
	volatile float delta;
	volatile float linearv;
} E_Scooter;

void CommPacket_Read(UART_HandleTypeDef* huart, uint8_t* buffer, uint8_t size, CommPacket*);

int IS_UART_BuildComm(CommPacket*);

int IS_UART_DatamapScooter(CommPacket*, E_Scooter*);

bool IS_UART_checkAsciidigit(uint8_t);

#endif /* INC_IS_UART_COMM_H_ */
