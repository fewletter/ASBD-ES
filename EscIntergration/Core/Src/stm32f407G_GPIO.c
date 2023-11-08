/*
 * stm32f407G_GPIO.c
 *
 *  Created on: Nov 6, 2023
 *      Author: fewletter
 */

#include "stm32f407G_GPIO.h"

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{

}

void GPIO_DeInit(GPIO_RegDef *pGPIOx);

/**
  * @brief  PeriClockControl
  * @param  pGIOx: port A...I
  * @param  state: ENABLE or DISABLE
  * @retval none
  */
void GPIO_PeriClockControl(GPIO_RegDef *pGIOx, uint8_t state)
{
	/* state == 1*/
	if (state) {
		switch (pGIOx) {
		    case GPIOA:
				GPIOA_PCLK_EN();
		    case GPIOB:
		        GPIOB_PCLK_EN();
		    case GPIOC:
                GPIOC_PCLK_EN();
		    case GPIOD:
		    	GPIOD_PCLK_EN();
		    case GPIOE:
				GPIOE_PCLK_EN();
		    case GPIOF:
		        GPIOF_PCLK_EN();
		    case GPIOG:
                GPIOG_PCLK_EN();
		    case GPIOH:
		    	GPIOH_PCLK_EN();
		    case GPIOI:
		    	GPIOI_PCLK_EN();
		}
	}
	else {
		switch (pGIOx) {
			case GPIOA:
				GPIOA_PCLK_DS();
			case GPIOB:
				GPIOB_PCLK_DS();
			case GPIOC:
				GPIOC_PCLK_DS();
			case GPIOD:
				GPIOD_PCLK_DS();
			case GPIOE:
				GPIOE_PCLK_DS();
			case GPIOF:
				GPIOF_PCLK_DS();
			case GPIOG:
				GPIOG_PCLK_DS();
			case GPIOH:
				GPIOH_PCLK_DS();
			case GPIOI:
				GPIOI_PCLK_DS();
		}
	}
}

void GPIO_ReadInputPin(GPIO_RegDef *pGPIOx, uint8_t PinNumber);
void GPIO_ReadInputPort(GPIO_RegDef *pGPIOx);
void GPIO_WriteOutputPin(GPIO_RegDef *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteOutputPort(GPIO_RegDef *pGPIOx, uint8_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef *pGPIOx, uint8_t PinNumber);

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t state);
void GPIO_IRQHandling(uint8_t PinNumber);
