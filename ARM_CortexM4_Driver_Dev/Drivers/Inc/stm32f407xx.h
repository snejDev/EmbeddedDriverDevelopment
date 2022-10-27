/*
 * stm32f407xx.h
 *
 *  Created on: Aug 22, 2022
 *  Author: snejdev
 */

#include <stdint.h>
#include <stddef.h>

#define __vo				volatile

#ifndef STM32F407XX_H_
#define STM32F407XX_H_

//Defining Base addresses of memories
#define FLASH_BASEADDR		0x08000000U					//Flash memory
#define SRAM1_BASEADDR		0x20000000U					//SRAM1
#define SRAM2_BASEADDR  	SRAM1_BASEADDR+0x1C000		//SRAM1+offset(size of SRAM1)
#define ROM_BASEADDR		0x1FFF0000U					//System memory
#define SRAM				SRAM1_BASEADDR				//Alias

//Defining Bus domain addresses
#define AHB1PERIPH_BASE		0x40020000U					//AHB1 Base address
#define AHB2PERIPH_BASE		0x50000000U					//AHB2 Base address
#define AHB3PERIPH_BASE		0xA0000000U					//AHB3 Base address
#define APB1PERIPH_BASE		0x40000000U					//APB1 Base address
#define APB2PERIPH_BASE		0x40010000U					//APB2 Base address

//NVIC Registers
//ISER Registers
#define NVIC_ISER0			((__vo uint32_t*)0xE000E100)	//ISER0 Address
#define NVIC_ISER1			((__vo uint32_t*)0xE000E104)	//ISER1 Address
#define NVIC_ISER2			((__vo uint32_t*)0xE000E108)	//ISER2 Address
#define NVIC_ISER3			((__vo uint32_t*)0xE000E10C)	//ISER3 Address
//ICER Registers
#define NVIC_ICER0			((__vo uint32_t*)0xE000E100)	//ICER0 Address
#define NVIC_ICER1			((__vo uint32_t*)0xE000E104)	//ICER1 Address
#define NVIC_ICER2			((__vo uint32_t*)0xE000E108)	//ICER2 Address
#define NVIC_ICER3			((__vo uint32_t*)0xE000E10C)	//ICER3 Address
//IPR Register - Pointer to Base Address
#define NVIC_IPR_BASE		((__vo uint32_t*)0xE000E400)			//IPR Base Address

//IRQ Number Macros
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

//IRQ Priority Macros
#define IRQ_PRI_15			15
#define IRQ_PRI_14			14
#define IRQ_PRI_13			13
#define IRQ_PRI_12			12
#define IRQ_PRI_11			11
#define IRQ_PRI_10			10
#define IRQ_PRI_9			9
#define IRQ_PRI_8			8
#define IRQ_PRI_7			7
#define IRQ_PRI_6			6
#define IRQ_PRI_5			5
#define IRQ_PRI_4			4
#define IRQ_PRI_3			3
#define IRQ_PRI_2			2
#define IRQ_PRI_1			1
#define IRQ_PRI_0			0


//Defining Base addresses for peripherals
//AHB1 Bus - GPIO Ports
#define GPIOA_BASEADDR		AHB1PERIPH_BASE				//GPIOA Baseaddress
#define GPIOB_BASEADDR		(AHB1PERIPH_BASE+0x0400)	//GPIOB Baseaddress
#define GPIOC_BASEADDR		(AHB1PERIPH_BASE+0x0800)	//GPIOC Baseaddress
#define GPIOD_BASEADDR		(AHB1PERIPH_BASE+0x0C00)	//GPIOD Baseaddress
#define GPIOE_BASEADDR		(AHB1PERIPH_BASE+0x1000)	//GPIOE Baseaddress
#define GPIOF_BASEADDR		(AHB1PERIPH_BASE+0x1400)	//GPIOF Baseaddress
#define GPIOG_BASEADDR		(AHB1PERIPH_BASE+0x1800)	//GPIOG Baseaddress
#define GPIOH_BASEADDR		(AHB1PERIPH_BASE+0x1C00)	//GPIOH Baseaddress
#define GPIOI_BASEADDR		(AHB1PERIPH_BASE+0x2000)	//GPIOI Baseaddress
#define GPIOJ_BASEADDR		(AHB1PERIPH_BASE+0x2400)	//GPIOJ Baseaddress
#define GPIOK_BASEADDR		(AHB1PERIPH_BASE+0x2800)	//GPIOK Baseaddress
#define RCC_BASEADDR 		(AHB1PERIPH_BASE+0x3800)	//RCC Baseaddress

//APB1 Bus
#define UART8_BASEADDR		(APB1PERIPH_BASE+0x7C00)	//UART8 Baseaddress
#define UART7_BASEADDR		(APB1PERIPH_BASE+0x7800)	//UART7 Baseaddress
#define I2C3_BASEADDR		(APB1PERIPH_BASE+0x5C00)	//I2C3 Baseaddress
#define I2C2_BASEADDR		(APB1PERIPH_BASE+0x5800)	//I2C2 Baseaddress
#define I2C1_BASEADDR		(APB1PERIPH_BASE+0x5400)	//I2C1 Baseaddress
#define UART5_BASEADDR		(APB1PERIPH_BASE+0x5000)	//UART5 Baseaddress
#define UART4_BASEADDR		(APB1PERIPH_BASE+0x4C00)	//UART4 Baseaddress
#define USART3_BASEADDR		(APB1PERIPH_BASE+0x4800)	//USART3 Baseaddress
#define USART2_BASEADDR		(APB1PERIPH_BASE+0x4400)	//USART2 Baseaddress
#define SPI3_BASEADDR		(APB1PERIPH_BASE+0x3C00)	//SPI3 Baseaddress
#define SPI2_BASEADDR		(APB1PERIPH_BASE+0x3800)	//SPI2 Baseaddress

//APB2 Bus
#define USART1_BASEADDR		(APB2PERIPH_BASE+0x1000)	//USART1 Baseaddress
#define USART6_BASEADDR		(APB2PERIPH_BASE+0x1400)	//USART6 Baseaddress
#define SPI1_BASEADDR		(APB2PERIPH_BASE+0x3000)	//SPI1 Baseaddress
#define SPI4_BASEADDR		(APB2PERIPH_BASE+0x3400)	//SPI4 Baseaddress
#define SYSCFG_BASEADDR		(APB2PERIPH_BASE+0x3800)	//SYSCFG Baseaddress
#define EXTI_BASEADDR		(APB2PERIPH_BASE+0x3C00)	//EXTI Baseaddress

//GPIO Register Definition structure
typedef struct
{								//Address offsets
	__vo uint32_t MODER;		//0x00
	__vo uint32_t OTYPER;		//0x04
	__vo uint32_t OSPEEDR;		//0x08
	__vo uint32_t PUPDR;		//0x0C
	__vo uint32_t IDR;			//0x10
	__vo uint32_t ODR;			//0x14
	__vo uint32_t BSRR;			//0x18
	__vo uint32_t LCKR;			//0x1C
	__vo uint32_t AF[2];		//0x20
}GPIO_RegDef_t;

//RCC Register Definition structure
typedef struct
{									//Address offsets
	__vo uint32_t CR;				//0x00
	__vo uint32_t PLLCFGR;			//0x04
	__vo uint32_t CFGR;				//0x08
	__vo uint32_t CIR;				//0x0C
	__vo uint32_t AHB1RSTR;			//0x10
	__vo uint32_t AHB2RSTR;			//0x14
	__vo uint32_t AHB3RSTR;			//0x18
	__vo uint32_t RESERVED;			//0x1C
	__vo uint32_t APB1RSTR;			//0x20
	__vo uint32_t APB2RSTR;			//0X24
	__vo uint32_t RESERVED1[2];		//0x28,0x2C
	__vo uint32_t AHB1ENR;			//0x30
	__vo uint32_t AHB2ENR;			//0x34
	__vo uint32_t AHB3ENR;			//0x38
	__vo uint32_t RESERVED2;		//0x3C
	__vo uint32_t APB1ENR;			//0x40
	__vo uint32_t APB2ENR;			//0x44
	__vo uint32_t RESERVED3[2];		//0X48,0x4C
	__vo uint32_t AHB1LPENR;		//0x50
	__vo uint32_t AHB2LPENR;		//0x54
	__vo uint32_t AHB3LPENR;		//0x58
	__vo uint32_t RESERVED4;		//0x5C
	__vo uint32_t APB1LPENR;		//0x60
	__vo uint32_t APB2LPENR;		//0x64
	__vo uint32_t RESERVED5[2];		//0x68,0x6C
	__vo uint32_t BDCR;				//0x70
	__vo uint32_t CSR;				//0x74
	__vo uint32_t RESERVED6[2];		//0x78,0x7C
	__vo uint32_t SSCGR;			//0x80
	__vo uint32_t PLLI2SCFGR;		//0x84
	//__vo uint32_t RCC_PLLSAICFGR;
	//__vo uint32_t RCC_DCKCFGR;
}RCC_RegDef_t;

//SYSCFG Register Definition Structure
typedef struct
{										//Address offsets
	__vo uint32_t SYSCFG_MEMRMP;		//0x00
	__vo uint32_t SYSCFG_PMC;			//0x04
	__vo uint32_t SYSCFG_EXTICR[4]; 	//0x08,0x0C,0x10,0x14
	__vo uint32_t SYSCFG_RESERVED1[2];	//0x18, 0x1C
	__vo uint32_t SYSCFG_CMPCR;			//0x20
	__vo uint32_t SYSCFG_RESERVED2[2];	//0x24, 0x28
	__vo uint32_t SYSCFG_CFGR;			//0x2C
}SYSCFG_RegDef_t;

//EXTI Register Definition Structure
typedef struct
{
	__vo uint32_t EXTI_IMR;			//Address offsets
	__vo uint32_t EXTI_EMR;			//0x00
	__vo uint32_t EXTI_RTSR;		//0x04
	__vo uint32_t EXTI_FTSR;		//0x08
	__vo uint32_t EXTI_SWIER;		//0x0C
	__vo uint32_t EXTI_PR;			//0x10
}EXTI_RegDef_t;

//GPIO Register macro definition
#define GPIOA				((GPIO_RegDef_t*)GPIOA_BASEADDR)	//Storing Typecasted base address of each GPIOA port Base address
#define GPIOB				((GPIO_RegDef_t*)GPIOB_BASEADDR)	//Storing Typecasted base address of each GPIOB port Base address
#define GPIOC				((GPIO_RegDef_t*)GPIOC_BASEADDR)	//Storing Typecasted base address of each GPIOC port Base address
#define GPIOD				((GPIO_RegDef_t*)GPIOD_BASEADDR)	//Storing Typecasted base address of each GPIOD port Base address
#define GPIOE				((GPIO_RegDef_t*)GPIOE_BASEADDR)	//Storing Typecasted base address of each GPIOE port Base address
#define GPIOF				((GPIO_RegDef_t*)GPIOF_BASEADDR)	//Storing Typecasted base address of each GPIOF port Base address
#define GPIOG				((GPIO_RegDef_t*)GPIOG_BASEADDR)	//Storing Typecasted base address of each GPIOG port Base address
#define GPIOH				((GPIO_RegDef_t*)GPIOH_BASEADDR)	//Storing Typecasted base address of each GPIOH port Base address
#define GPIOI				((GPIO_RegDef_t*)GPIOI_BASEADDR)	//Storing Typecasted base address of each GPIOI port Base address

//RCC Register macro definition
#define RCCCLK				((RCC_RegDef_t*)RCC_BASEADDR) 		//Storing Typecasted base address of RCC clock Base address

//EXTI Register macro definition
#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)		//Storing Typecasted base address of each EXTI Base address
//SYSCFG Register macro Definition
#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR) //Storing Typecasted base address of each EXTI Base address

/***Peripheral clocks enable and disable***/
//GPIOx Peripheral clock enable
#define GPIOA_PCLK_EN()		(RCCCLK->AHB1ENR |= (0x01<<0))
#define GPIOB_PCLK_EN()		(RCCCLK->AHB1ENR |= (0x01<<1))
#define GPIOC_PCLK_EN()		(RCCCLK->AHB1ENR |= (0x01<<2))
#define GPIOD_PCLK_EN()		(RCCCLK->AHB1ENR |= (0x01<<3))
#define GPIOE_PCLK_EN()		(RCCCLK->AHB1ENR |= (0x01<<4))
#define GPIOF_PCLK_EN()		(RCCCLK->AHB1ENR |= (0x01<<5))
#define GPIOG_PCLK_EN()		(RCCCLK->AHB1ENR |= (0x01<<6))
#define GPIOH_PCLK_EN()		(RCCCLK->AHB1ENR |= (0x01<<7))
#define GPIOI_PCLK_EN()		(RCCCLK->AHB1ENR |= (0x01<<8))

//GPIOx Peripheral clock disable
#define GPIOA_PCLK_DI()		(RCCCLK->AHB1ENR &= ~(0x01<<0))
#define GPIOB_PCLK_DI()		(RCCCLK->AHB1ENR &= ~(0x01<<1))
#define GPIOC_PCLK_DI()		(RCCCLK->AHB1ENR &= ~(0x01<<2))
#define GPIOD_PCLK_DI()		(RCCCLK->AHB1ENR &= ~(0x01<<3))
#define GPIOE_PCLK_DI()		(RCCCLK->AHB1ENR &= ~(0x01<<4))
#define GPIOF_PCLK_DI()		(RCCCLK->AHB1ENR &= ~(0x01<<5))
#define GPIOG_PCLK_DI()		(RCCCLK->AHB1ENR &= ~(0x01<<6))
#define GPIOH_PCLK_DI()		(RCCCLK->AHB1ENR &= ~(0x01<<7))
#define GPIOI_PCLK_DI()		(RCCCLK->AHB1ENR &= ~(0x01<<8))

//SYSCFG Clock Enable
#define SYSCFG_CLK_EN()		(RCCCLK->APB2ENR |= (1<<14))
/***Peripheral clocks enable and disable***/

#define ENABLE		1
#define DISABLE		0
#define SET			1
#define RESET 		0

//NVIC_IRP Register
#define IMPLEMENTED_BITS	4

#include "stm32f407xx_gpio_drv.h"

#endif /* STM32F407XX_H_ */
