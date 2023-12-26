/*
 * PS2.c
 *
 *  Created on: Dec 14, 2023
 *      Author: fewletter
 */

#ifndef SRC_PS2_C_
#define SRC_PS2_C_

#include "PS2.h"
#include <string.h>
#include <stdio.h>

extern void PS2_Init(PS2_State* PS2)
{
	PS2->Button = 0;
	memset(PS2->Lstick, 0, sizeof(uint8_t)*2);
	memset(PS2->Rstick, 0, sizeof(uint8_t)*2);
};

bool PS2_checkAsciiHex(uint8_t digit)
{
	if ((digit - '0') >= 0 && (digit - '9') <= 0)
		return true;
	else if ((digit - 'A') >= 0 && (digit - 'F') <= 0)
		return true;
	else
		return false;
};

uint8_t PS2_mapAsciiuint8(uint8_t digit)
{
	return (digit - '0' > 9) ? (10 + digit - 'A') : (digit - '0');
};

extern void PS2_ReadData(PS2_State* PS2, UART_HandleTypeDef *huart, uint8_t* buffer)
{
	int word_count = 0;
	HAL_UART_Receive_DMA(huart, buffer, 14);
	//printf("%d %d %d %d\n", PS2->Lstick[0], PS2->Lstick[1], PS2->Rstick[0], PS2->Rstick[1]);
}

#endif /* SRC_PS2_C_ */
