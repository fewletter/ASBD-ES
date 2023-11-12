/*
 * stm32f407G.h
 *
 *  Created on: Nov 6, 2023
 *      Author: fewletter
 */

#ifndef INC_STM32F407G_H_
#define INC_STM32F407G_H_

#include <stdint.h>

#define __vo volatile

#define FLASH_BASEADDR       0x08000000U   /*Flash memory address  0x08000000 ~ 0x080fffff*/
#define SRAM1_BASEADDR       0x20000000U   /*SRAM memory address 112 kb*/
#define SRAM2_BASEADDR       0x20001c00U   /*SRAM2 memory address 16 kb*/
#define ROM_BASEADDR         0x1fff0000U   /*System memory address*/
#define SRAM                 SRAM1_BASEADDR

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR      0x40000000U
#define APB1PERIPH_BASEADDR  PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR  0x40010000U
#define AHB1PERIPH_BASEADDR  0x40020000U
#define AHB2PERIPH_BASEADDR  0x50000000U

/*
 * AHB1 bus
 */
#define GPIOA_BASEADDR       (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR       (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR       (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR       (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR       (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR       (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR       (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR       (AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR       (AHB1PERIPH_BASEADDR + 0x2000)
#define RCC_BASEADDR         (AHB1PERIPH_BASEADDR + 0x3800)

/*
 * APB1 bus
 */
#define I2C1_BASEADDR        (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR        (APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR        (APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR        (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR        (APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR      (APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR      (APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR       (APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR       (APB1PERIPH_BASEADDR + 0x5000)

/*
 * APB2 bus
 */
#define EXTI_BASEADDR        (APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR        (APB2PERIPH_BASEADDR + 0x3000)

/*
 * GPIO
 */
typedef struct
{
	__vo uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
	__vo uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
	__vo uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
	__vo uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
	__vo uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
	__vo uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
	__vo uint32_t BSRR;     /*!< GPIO port bit set/reset register,      Address offset: 0x18      */
	__vo uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
	__vo uint32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
}GPIO_RegDef;

/*
 * RCC
 */
typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	uint32_t RESERVED2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	uint32_t RESERVED4;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t RESERVED6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
}RCC_RegDef;

#define GPIOA                ((GPIO_RegDef*)GPIOA_BASEADDR)
#define GPIOB                ((GPIO_RegDef*)GPIOB_BASEADDR)
#define GPIOC                ((GPIO_RegDef*)GPIOC_BASEADDR)
#define GPIOD                ((GPIO_RegDef*)GPIOD_BASEADDR)
#define GPIOE                ((GPIO_RegDef*)GPIOE_BASEADDR)
#define GPIOF                ((GPIO_RegDef*)GPIOF_BASEADDR)
#define GPIOG                ((GPIO_RegDef*)GPIOG_BASEADDR)
#define GPIOH                ((GPIO_RegDef*)GPIOH_BASEADDR)
#define GPIOI                ((GPIO_RegDef*)GPIOI_BASEADDR)

#define RCC                  ((RCC_RegDef*)RCC_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals | manual p.242
 */
#define GPIOA_PCLK_EN()  (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()  (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()  (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()  (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()  (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()  (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()  (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()  (RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()  (RCC->AHB1ENR |= (1 << 8))

/*
 * Clock Enable Macros for I2Cx peripherals | manual p.174
 */
#define I2C1_PCLK_EN()    (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()    (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()    (RCC->APB1ENR |= (1 << 23))

/*
 * Clock Enable Macros for SPIx peripherals | manual p.178,174
 */
#define SPI1_PCLK_EN()    (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()    (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()    (RCC->APB1ENR |= (1 << 15))

/*
 * Clock Enable Macros for USARTx peripherals | manual p.178,174
 */
#define USART1_PCLK_EN()  (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()  (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()  (RCC->APB1ENR |= (1 << 18))

/*
 * Clock Disable Macros for GPIOx peripherals | manual p.242
 */
#define GPIOA_PCLK_DS()  (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DS()  (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DS()  (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DS()  (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DS()  (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DS()  (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DS()  (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DS()  (RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DS()  (RCC->AHB1ENR &= ~(1 << 8))

/*
 * Reset GPIOx perpipherals
 */
#define GPIOA_REG_RESET()                  \
		do {(RCC->AHB1RSTR |= (1 << 0));   \
		    (RCC->AHB1RSTR &= ~(1 << 0));} \
		while(0)

#define GPIOB_REG_RESET()                  \
		do {(RCC->AHB1RSTR |= (1 << 1));   \
		    (RCC->AHB1RSTR &= ~(1 << 1));} \
		while(0)

#define GPIOC_REG_RESET()                  \
		do {(RCC->AHB1RSTR |= (1 << 2));   \
		    (RCC->AHB1RSTR &= ~(1 << 2));} \
		while(0)

#define GPIOD_REG_RESET()                  \
		do {(RCC->AHB1RSTR |= (1 << 3));   \
		    (RCC->AHB1RSTR &= ~(1 << 3));} \
		while(0)

#define GPIOE_REG_RESET()                  \
		do {(RCC->AHB1RSTR |= (1 << 4));   \
		    (RCC->AHB1RSTR &= ~(1 << 4));} \
		while(0)

#define GPIOF_REG_RESET()                  \
		do {(RCC->AHB1RSTR |= (1 << 5));   \
		    (RCC->AHB1RSTR &= ~(1 << 5));} \
		while(0)

#define GPIOG_REG_RESET()                  \
		do {(RCC->AHB1RSTR |= (1 << 6));   \
		    (RCC->AHB1RSTR &= ~(1 << 6));} \
		while(0)

#define GPIOH_REG_RESET()                  \
		do {(RCC->AHB1RSTR |= (1 << 7));   \
		    (RCC->AHB1RSTR &= ~(1 << 7));} \
		while(0)

#define GPIOI_REG_RESET()                  \
		do {(RCC->AHB1RSTR |= (1 << 8));   \
		    (RCC->AHB1RSTR &= ~(1 << 8));} \
		while(0)


/*
 * General setup
 */
#define ENABLE           1
#define DISABLE          0
#define SET              ENABLE
#define RESET            DISABLE
#define GPIO_PIN_SET     SET
#define GPIO_PIN_RESET   RESET

#endif /* INC_STM32F407G_H_ */
