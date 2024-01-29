/*
 * RWheel.h
 *
 *  Created on: Jan 16, 2024
 *      Author: fewletter
 */

#ifndef INC_RWHEEL_H_
#define INC_RWHEEL_H_

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include <math.h>
#include "IS_UART_Comm.h"

#define ON  0
#define OFF 1

/*
 * START_STOP  -> Black
 * CW_CCW      -> Gray
 * RUN_BRAKE   -> White
 * INT_EXT     -> Light Blue
 * ALARM       -> Brown
 * ALARM_RESET -> Purple
 * SPEED       -> Red
 * GND         -> Orange
 */
typedef struct {
	bool ALARM;
	//GPIO_TypeDef *alram;
	//uint16_t alarm_pin;
	uint16_t SPEED;
	bool GND;
	bool VRL;
	bool VRM;
	bool VRH;
	bool ALARM_RESET;
	bool INT_EXT;
	bool CW_CCW;
	bool RUN_BRAKE;
	bool START_STOP;
} Signal_state;

void RW_motor_init(void);

/*
 * PD0 -> Black
 * PD1 -> Gray
 * PD2 -> White
 * PA8 -> Light Blue
 * PC8 -> Brown
 * PC9 -> Purple
 * PB0 -> Red
 * GND -> Orange
 */
void RW_pin_config(void);
uint8_t RW_PDtorque(E_Scooter*, uint8_t, uint8_t);

#endif /* INC_RWHEEL_H_ */
