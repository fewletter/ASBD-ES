/*
 * stm32f407G_GPIO.H
 *
 *  Created on: Nov 8, 2023
 *      Author: fewletter
 */

#ifndef INC_STM32F407G_GPIO_H_
#define INC_STM32F407G_GPIO_H_

#include "stm32f407G.h"

typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_Config_t;

typedef struct
{
	GPIO_RegDef   *pGPIOx;
	GPIO_Config_t GPIO_PinConfig;
}GPIO_Handle_t;

/*
 * @GPIO_PinNumber
 */
#define GPIO_PIN_0                 0  /* Pin 0 selected    */
#define GPIO_PIN_1                 1  /* Pin 1 selected    */
#define GPIO_PIN_2                 2  /* Pin 2 selected    */
#define GPIO_PIN_3                 3  /* Pin 3 selected    */
#define GPIO_PIN_4                 4  /* Pin 4 selected    */
#define GPIO_PIN_5                 5  /* Pin 5 selected    */
#define GPIO_PIN_6                 6  /* Pin 6 selected    */
#define GPIO_PIN_7                 7  /* Pin 7 selected    */
#define GPIO_PIN_8                 8  /* Pin 8 selected    */
#define GPIO_PIN_9                 9  /* Pin 9 selected    */
#define GPIO_PIN_10                10  /* Pin 10 selected   */
#define GPIO_PIN_11                11  /* Pin 11 selected   */
#define GPIO_PIN_12                12  /* Pin 12 selected   */
#define GPIO_PIN_13                13  /* Pin 13 selected   */
#define GPIO_PIN_14                14  /* Pin 14 selected   */
#define GPIO_PIN_15                15  /* Pin 15 selected   */

/*
 * @GPIO_PinMode
 * Pin possible modes
 */
#define GPIO_MODE_IN       0
#define GPIO_MODE_OUT      1
#define GPIO_MODE_ALTFN    2
#define GPIO_MODE_ANALOG   3
#define GPIO_MODE_IT_FT    4
#define GPIO_MODE_IT_RT    5
#define GPIO_MODE_IT_RFT   6

/*
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP    0
#define GPIO_OP_TYPE_OD    1

/*
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW     0
#define GPIO_SPEED_MEDIUM  1
#define GPIO_SPEED_FAST    2
#define GPIO_SPEED_HIGH    3

/*
 * GPIO pin pull up & pull down configuration
 */
#define GPIO_NO_PUPD       0
#define GPIO_PU            1
#define GPIO_PD            2

/*
 * GPIO API
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef *pGPIOx);

void GPIO_PeriClockControl(GPIO_RegDef *pGIOx, uint8_t state);

uint8_t GPIO_ReadInputPin(GPIO_RegDef *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadInputPort(GPIO_RegDef *pGPIOx);
void GPIO_WriteOutputPin(GPIO_RegDef *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteOutputPort(GPIO_RegDef *pGPIOx, uint8_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef *pGPIOx, uint8_t PinNumber);

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t state);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F407G_GPIO_H_ */
