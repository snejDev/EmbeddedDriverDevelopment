/*
 * stm32f407xx.h
 *
 *  Created on: Aug 22, 2022
 *  Author: snejdev
 */

#include <stdint.h>

#define _vo					volatile;

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

//GPIO Register Definitions
#define GPIOA				(GPIO_RegDef*)GPIOA_BASEADDR	//Storing Typecasted base address of each GPIOA port Base address
#define GPIOB				(GPIO_RegDef*)GPIOB_BASEADDR	//Storing Typecasted base address of each GPIOB port Base address
#define GPIOC				(GPIO_RegDef*)GPIOC_BASEADDR	//Storing Typecasted base address of each GPIOC port Base address
#define GPIOD				(GPIO_RegDef*)GPIOD_BASEADDR	//Storing Typecasted base address of each GPIOD port Base address
#define GPIOE				(GPIO_RegDef*)GPIOE_BASEADDR	//Storing Typecasted base address of each GPIOE port Base address
#define GPIOF				(GPIO_RegDef*)GPIOF_BASEADDR	//Storing Typecasted base address of each GPIOF port Base address
#define GPIOG				(GPIO_RegDef*)GPIOG_BASEADDR	//Storing Typecasted base address of each GPIOG port Base address
#define GPIOH				(GPIO_RegDef*)GPIOH_BASEADDR	//Storing Typecasted base address of each GPIOH port Base address
#define GPIOI				(GPIO_RegDef*)GPIOI_BASEADDR	//Storing Typecasted base address of each GPIOI port Base address

//RCC Clock Register macro definition
#define RCCCLK				(RCC_RegDef*)RCC_BASEADDR;		//Storing Typecasted base address of each RCC clock Base address

//Clock enable and disable macros


//GPIO register definition structure
typedef struct
{								//Address offsets
	_vo uint32_t MODER;			//0x00
	_vo uint32_t OTYPER;		//0x04
	_vo uint32_t OSPEEDR;		//0x08
	_vo uint32_t PUPDR;			//0x0C
	_vo uint32_t IDR;			//0x10
	_vo uint32_t ODR;			//0x14
	_vo uint32_t BSRR;			//0x18
	_vo uint32_t LCKR;			//0x1C
	_vo uint32_t AF[2];			//0x20
}GPIO_RegDef_t;

typedef struct
{									//Address offsets
	_vo uint32_t RCC_CR;			//0x00
	_vo	uint32_t RCC_PLLCFGR;		//0x04
	_vo	uint32_t RCC_CFGR;			//0x08
	_vo	uint32_t RCC_CIR;			//0x0C
	_vo	uint32_t RCC_AHB1RSTR;		//0x10
	_vo	uint32_t RCC_AHB2RSTR;		//0x14
	_vo uint32_t RCC_AHB3RSTR;		//0x18
	_vo uint32_t RESERVED;			//0x1C
	_vo uint32_t RCC_APB1RSTR;		//0x20
	_vo uint32_t RCC_APB2RSTR;		//0X24
	_vo uint32_t RESERVED1[2];		//0x28,0x2C
	_vo uint32_t RCC_AHB1ENR;		//0x30
	_vo uint32_t RCC_AHB2ENR;		//0x34
	_vo uint32_t RCC_AHB3ENR;		//0x38
	_vo uint32_t RESERVED2;			//0x3C
	_vo uint32_t RCC_APB1ENR;		//0x40
	_vo uint32_t RCC_APB2ENR;		//0x44
	_vo uint32_t RESERVED3[2];		//0X48,0x4C
	_vo uint32_t RCC_AHB1LPENR;		//0x50
	_vo uint32_t RCC_AHB2LPENR;		//0x54
	_vo uint32_t RCC_AHB3LPENR;		//0x58
	_vo uint32_t RESERVED4;			//0x5C
	_vo uint32_t RCC_APB1LPENR;		//0x60
	_vo uint32_t RCC_APB2LPENR;		//0x64
	_vo uint32_t RESERVED5[2];		//0x68,0x6C
	_vo uint32_t RCC_BDCR;			//0x70
	_vo uint32_t RCC_CSR;			//0x74
	_vo uint32_t RESERVED6[2];		//0x78,0x7C
	_vo uint32_t RCC_SSCGR;			//0x80
	_vo uint32_t RCC_PLLI2SCFGR;	//0x84
	//_vo uint32_t RCC_PLLSAICFGR;
	//_vo uint32_t RCC_DCKCFGR;
}RCC_RegDef_t;

//Peripheral clocks enable and disable

//GPIOx Peripheral clock enable
#define GPIOA_PCLK_EN()		RCCCLK->RCC_AHB1ENR |= (0x01<<0);
#define GPIOB_PCLK_EN()		RCCCLK->RCC_AHB1ENR |= (0x01<<1);
#define GPIOC_PCLK_EN()		RCCCLK->RCC_AHB1ENR |= (0x01<<2);
#define GPIOD_PCLK_EN()		RCCCLK->RCC_AHB1ENR |= (0x01<<3);
#define GPIOE_PCLK_EN()		RCCCLK->RCC_AHB1ENR |= (0x01<<4);
#define GPIOF_PCLK_EN()		RCCCLK->RCC_AHB1ENR |= (0x01<<5);
#define GPIOG_PCLK_EN()		RCCCLK->RCC_AHB1ENR |= (0x01<<6);
#define GPIOH_PCLK_EN()		RCCCLK->RCC_AHB1ENR |= (0x01<<7);
#define GPIOI_PCLK_EN()		RCCCLK->RCC_AHB1ENR |= (0x01<<8);

//GPIOx Peripheral clock disable
#define GPIOA_PCLK_DI()		RCCCLK->RCC_AHB1ENR &= ~(0x01<<0);
#define GPIOB_PCLK_DI()		RCCCLK->RCC_AHB1ENR &= ~(0x01<<1);
#define GPIOC_PCLK_DI()		RCCCLK->RCC_AHB1ENR &= ~(0x01<<2);
#define GPIOD_PCLK_DI()		RCCCLK->RCC_AHB1ENR &= ~(0x01<<3);
#define GPIOE_PCLK_DI()		RCCCLK->RCC_AHB1ENR &= ~(0x01<<4);
#define GPIOF_PCLK_DI()		RCCCLK->RCC_AHB1ENR &= ~(0x01<<5);
#define GPIOG_PCLK_DI()		RCCCLK->RCC_AHB1ENR &= ~(0x01<<6);
#define GPIOH_PCLK_DI()		RCCCLK->RCC_AHB1ENR &= ~(0x01<<7);
#define GPIOI_PCLK_DI()		RCCCLK->RCC_AHB1ENR &= ~(0x01<<8);


#define ENABLE		1
#define DISABLE		0
#define SET			1
#define RESET 		0


#endif /* STM32F407XX_H_ */
