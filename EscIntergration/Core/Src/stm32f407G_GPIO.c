/*
 * stm32f407G_GPIO.c
 *
 *  Created on: Nov 6, 2023
 *      Author: fewletter
 */

#include "stm32f407G_GPIO.h"

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    uint32_t temp = 0;

    /*
     * Configure mode of gpio pin
     */
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
    	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    	pGPIOHandle->pGPIOx->MODER &= ~(0X3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    	pGPIOHandle->pGPIOx->MODER |= temp;
    }
    else {

    }

    temp = 0;

    /*
	 * Configure the speed
	 */
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0X3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OSPEEDR |= temp;
    temp = 0;

    /*
	 * Configure the PUPD
	 */
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0X3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	/*
	 * Configure the optype
	 */
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0X1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	/*
	 * Configure the alt function
	 */
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ALTFN) {
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0XF << (4*temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= ~(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));
	}
	else {

	}
}

void GPIO_DeInit(GPIO_RegDef *pGPIOx)
{
	if (pGPIOx == GPIOA)
		GPIOA_REG_RESET();
	else if (pGPIOx == GPIOB)
		GPIOB_REG_RESET();
	else if (pGPIOx == GPIOC)
		GPIOC_REG_RESET();
	else if (pGPIOx == GPIOD)
		GPIOD_REG_RESET();
	else if (pGPIOx == GPIOE)
		GPIOE_REG_RESET();
	else if (pGPIOx == GPIOF)
		GPIOF_REG_RESET();
	else if (pGPIOx == GPIOG)
		GPIOG_REG_RESET();
	else if (pGPIOx == GPIOH)
		GPIOH_REG_RESET();
	else if (pGPIOx == GPIOI)
		GPIOI_REG_RESET();

}

/**
  * @brief  PeriClockControl
  * @param  pGIOx: port A...I
  * @param  state: ENABLE or DISABLE
  * @retval none
  */
void GPIO_PeriClockControl(GPIO_RegDef *pGPIOx, uint8_t state)
{
	/* state == ENABLE*/
	if (state) {
		if (pGPIOx == GPIOA)
			GPIOA_PCLK_EN();
		else if (pGPIOx == GPIOB)
		    GPIOB_PCLK_EN();
		else if (pGPIOx == GPIOC)
			GPIOC_PCLK_EN();
		else if (pGPIOx == GPIOD)
		    GPIOD_PCLK_EN();
		else if (pGPIOx == GPIOE)
			GPIOE_PCLK_EN();
		else if (pGPIOx == GPIOF)
		    GPIOF_PCLK_EN();
		else if (pGPIOx == GPIOG)
			GPIOG_PCLK_EN();
		else if (pGPIOx == GPIOH)
		    GPIOH_PCLK_EN();
		else if (pGPIOx == GPIOI)
			GPIOI_PCLK_EN();
	}
	else {
		if (pGPIOx == GPIOA)
			GPIOA_PCLK_DS();
		else if (pGPIOx == GPIOB)
			GPIOB_PCLK_DS();
		else if (pGPIOx == GPIOC)
			GPIOC_PCLK_DS();
		else if (pGPIOx == GPIOD)
			GPIOD_PCLK_DS();
		else if (pGPIOx == GPIOE)
			GPIOE_PCLK_DS();
		else if (pGPIOx == GPIOF)
			GPIOF_PCLK_DS();
		else if (pGPIOx == GPIOG)
			GPIOG_PCLK_DS();
		else if (pGPIOx == GPIOH)
			GPIOH_PCLK_DS();
		else if (pGPIOx == GPIOI)
			GPIOI_PCLK_DS();
	}
}

/**
  * @brief  GPIO_ReadInputPin
  * @param  pGIOx: port A...I
  * @param  PinNumber: 1...15
  * @retval uint8_t
  */
uint8_t GPIO_ReadInputPin(GPIO_RegDef *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x1);
	return value;
}

/**
  * @brief  GPIO_ReadInputPort
  * @param  pGIOx: port A...I
  * @retval uint16_t
  */
uint16_t GPIO_ReadInputPort(GPIO_RegDef *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}

/**
  * @brief  GPIO_WriteOutputPin(
  * @param  pGIOx: port A...I
  * @retval uint16_t
  */
void GPIO_WriteOutputPin(GPIO_RegDef *pGPIOx, uint8_t PinNumber, uint8_t state)
{
	/* state == ENABLE*/
	if (state)
		pGPIOx->ODR |= (1 << PinNumber);
	else
		pGPIOx->ODR &= ~(1 << PinNumber);
}

/**
  * @brief  GPIO_WriteOutputPort
  * @param  pGIOx: port A...I
  * @retval uint16_t
  */
void GPIO_WriteOutputPort(GPIO_RegDef *pGPIOx, uint8_t value)
{
	pGPIOx->ODR = value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t state);
void GPIO_IRQHandling(uint8_t PinNumber);
