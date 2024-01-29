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

void PS2_new(PS2_State* PS2)
{
	PS2 = malloc(sizeof(PS2_State));
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

extern void PS2_ReadData(I2C_HandleTypeDef *hi2c, uint8_t* buffer)
{
	int word_count = 0;
	HAL_I2C_Slave_Receive(hi2c, buffer, 4, 10);
	//printf("%d %d %d %d\n", PS2->Lstick[0], PS2->Lstick[1], PS2->Rstick[0], PS2->Rstick[1]);
}

void PS2_DatamapScooter(uint16_t DAC_V, E_Scooter* state)
{
	/*
	 * Highest speed of E-scooter is 11km/hr -> 39.6m/s. Therefore the linearv's unit is m/s.
	 */
	state->linearv = 43.2/2495 * (DAC_V - 1600);
}

#endif /* SRC_PS2_C_ */
