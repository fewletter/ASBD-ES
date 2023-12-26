/*
 * PS2.h
 *
 *  Created on: Dec 14, 2023
 *      Author: fewletter
 */

#ifndef INC_PS2_H_
#define INC_PS2_H_
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "main.h"
#include "core_cm4.h"

#define START     0x01
#define READDATA  0x42

#define CLK_PIN   10
#define CS_PIN    11
#define DAT_PIN   12
#define CMD_PIN   13

typedef enum {
	PS2_START,
	PS2_SELECT,
	PS2_TOP,
	PS2_DOWN,
	PS2_LEFT,
	PS2_RIGHT,
	PS2_L3,
	PS2_R3,
	PS2_L2,
	PS2_R2,
	PS2_X,
	PS2_Y,
	PS2_A,
	PS2_B,
	PS2_L1,
	PS2_R1,
}PS2_Button;

typedef struct {
	PS2_Button Button;
	uint8_t Lstick[2];
	uint8_t Rstick[2];
}PS2_State;

extern void PS2_Init(PS2_State*);
extern void PS2_ReadData(PS2_State*, UART_HandleTypeDef*, uint8_t*);
bool PS2_checkAsciiHex(uint8_t);
uint8_t PS2_mapAsciiuint8(uint8_t digit);

#endif /* INC_PS2_H_ */
