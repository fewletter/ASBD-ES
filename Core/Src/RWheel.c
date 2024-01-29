/*
 * RWheel.c
 *
 *  Created on: Jan 16, 2024
 *      Author: fewletter
 */

#include "RWheel.h"

void RW_motor_init(void)
{
	/*
	 * START_STOP ON
	 */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, ON);

	/*
	 * RUN_BRAKE ON
	 */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, ON);

	/*
	 * CW_CCW ON
	 */
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, OFF);

	/*
	 * INT_EXT ON
	 */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, ON);
}

/*
 * Black, Gray, White, Light Blue, Brown, Purple, Red
 */
void RW_pin_config(void)
{

}


uint8_t RW_PDtorque(E_Scooter* state, uint8_t Kp, uint8_t Kd)
{
    uint8_t desired_theta = 0;
    uint8_t desired_dtheta = 0;
    uint8_t dtheta = state->theta_p1 - state->theta / 0.012;
    return Kp*(desired_theta - state->theta_p1) + Kd*(desired_dtheta - dtheta);
}
