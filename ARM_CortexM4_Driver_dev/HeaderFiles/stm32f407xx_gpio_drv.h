/*
 * stm32f404xx_gpio_drv.h
 *
 *  Created on: Oct 22, 2022
 *      Author: snejdev
 */

#ifndef STM32F404XX_GPIO_DRV_H_
#define STM32F404XX_GPIO_DRV_H_

#include "stm32f407xx.h"
#include <stdint.h>
#include <stddef.h>

void GPIO_PCLK_Ctrl(GPIO_RegDef_t *pGPIOx, uint8_t EN_DI);

/**Macros defining Pin configuration**/

//@PinNo
//GPIO_PinNumber
#define GPIO_PINNO_0	0
#define GPIO_PINNO_1	1
#define GPIO_PINNO_2	2
#define GPIO_PINNO_3	3
#define GPIO_PINNO_4	4
#define GPIO_PINNO_5	5
#define GPIO_PINNO_6	6
#define GPIO_PINNO_7	7
#define GPIO_PINNO_8	8
#define GPIO_PINNO_9	9
#define GPIO_PINNO_10	10
#define GPIO_PINNO_11	11
#define GPIO_PINNO_12	12
#define GPIO_PINNO_13	13
#define GPIO_PINNO_14	14
#define GPIO_PINNO_15	15

//GPIO_PinMode
//@PinMode
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_AF		2
#define GPIO_MODE_AN		3
#define GPIO_MODE_IT_RT		4
#define GPIO_MODE_IT_FT		5
#define GPIO_MODE_IT_RFT	6

//GPIO_PinSpeed
//@PinSpeed
#define GPIO_PINSPEED_LOW	0
#define GPIO_PINSPEED_MED	1
#define GPIO_PINSPEED_HIGH	2
#define GPIO_PINSPEED_VHIGH	3

//GPIO_PinPuPdCtrl
//@PinPuPdCtrl
#define GPIO_PIN_NPUPD	0
#define GPIO_PIN_PU		1
#define GPIO_PIN_PD		2

//GPIO_OPtype
//@PinOPtype
#define GPIO_PP	0
#define GPIO_OD	1

/**Macros defining Pin configuration**/

/**Function Macros to Reset GPIO pins**/

#define GPIOA_RST()		do{RCCCLK->AHB1RSTR |= 1<<0; RCCCLK->AHB1RSTR &= ~(1<<0);}while(0)
#define GPIOB_RST()		do{RCCCLK->AHB1RSTR |= 1<<1; RCCCLK->AHB1RSTR &= ~(1<<1);}while(0)
#define GPIOC_RST()		do{RCCCLK->AHB1RSTR |= 1<<2; RCCCLK->AHB1RSTR &= ~(1<<2);}while(0)
#define GPIOD_RST()		do{RCCCLK->AHB1RSTR |= 1<<3; RCCCLK->AHB1RSTR &= ~(1<<3);}while(0)
#define GPIOE_RST()		do{RCCCLK->AHB1RSTR |= 1<<4; RCCCLK->AHB1RSTR &= ~(1<<4);}while(0)
#define GPIOF_RST()		do{RCCCLK->AHB1RSTR |= 1<<5; RCCCLK->AHB1RSTR &= ~(1<<5);}while(0)
#define GPIOG_RST()		do{RCCCLK->AHB1RSTR |= 1<<6; RCCCLK->AHB1RSTR &= ~(1<<6);}while(0)
#define GPIOH_RST()		do{RCCCLK->AHB1RSTR |= 1<<7; RCCCLK->AHB1RSTR &= ~(1<<7);}while(0)
#define GPIOI_RST()		do{RCCCLK->AHB1RSTR |= 1<<8; RCCCLK->AHB1RSTR &= ~(1<<8);}while(0)

/**Function Macros to Reset GPIO pins**/

typedef struct
{								//Accepted inits
	uint8_t GPIO_PinNo;			//@PinNo
	uint8_t GPIO_PinMode;		//@PinMode
	uint8_t GPIO_PinSpeed;		//@PinSpeed
	uint8_t GPIO_PinPuPdCtrl;	//@PinPuPdCtrl
	uint8_t GPIO_PinOPtype;		//@PinOPtype
	uint8_t GPIO_PinAltFunc;
}GPIO_PinConfig_t;

//	GPIO API Handle and Configuration Structures
typedef struct
{
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIOx_Handle_t;

//	API Prototypes

/*
 * 	@function				: GPIO_PCLK_Ctrl
 * 	@info					: Peripheral Clock Control
 *
 * 	@param[in]_datatypes	: GPIO_RegDef_t, uint8_t
 * 	@param[in] variables	: GPIO_RegDef_t *pGPIO, uint8_t EN_DI
 *
 * 	@return					: void
 *
 * 	@notes					: API for enabling and disabling the clock
 */
void GPIO_PCLK_Ctrl(GPIO_RegDef_t *pGPIOx, uint8_t EN_DI)
{
	if(EN_DI==ENABLE)
	{
		if(pGPIOx==GPIOA)
			GPIOA_PCLK_EN();
		else if(pGPIOx==GPIOB)
			GPIOB_PCLK_EN();
		else if(pGPIOx==GPIOC)
			GPIOC_PCLK_EN();
		else if(pGPIOx==GPIOD)
			GPIOD_PCLK_EN();
		else if(pGPIOx==GPIOE)
			GPIOE_PCLK_EN();
		else if(pGPIOx==GPIOF)
			GPIOF_PCLK_EN();
		else if(pGPIOx==GPIOG)
			GPIOG_PCLK_EN();
		else if(pGPIOx==GPIOH)
			GPIOH_PCLK_EN();
		else if(pGPIOx==GPIOI)
			GPIOI_PCLK_EN();
	}
	else if(EN_DI==DISABLE)
	{
		if(pGPIOx==GPIOA)
			GPIOA_PCLK_DI();
		else if(pGPIOx==GPIOB)
			GPIOB_PCLK_DI();
		else if(pGPIOx==GPIOC)
			GPIOC_PCLK_DI();
		else if(pGPIOx==GPIOD)
			GPIOD_PCLK_DI();
		else if(pGPIOx==GPIOE)
			GPIOE_PCLK_DI();
		else if(pGPIOx==GPIOF)
			GPIOF_PCLK_DI();
		else if(pGPIOx==GPIOG)
			GPIOG_PCLK_DI();
		else if(pGPIOx==GPIOH)
			GPIOH_PCLK_DI();
		else if(pGPIOx==GPIOI)
			GPIOI_PCLK_DI();
	}
}

/*
 * 	@function				: GPIO_Init
 * 	@info					: Initializing the required GPIO pins
 *
 * 	@param[in]_datatypes	: GPIO_Handle_t
 * 	@param[in] variables	: GPIO_Handle_t *pGPIOx_Handle
 *
 * 	@return					: void
 *
 * 	@notes					: API for initializing the GPIO pins
 */
void GPIO_Init(GPIOx_Handle_t *pGPIOx_Handle)
{
	uint32_t temp;

	//Pin mode configuration
	if(pGPIOx_Handle->GPIO_PinConfig.GPIO_PinMode<=GPIO_MODE_AN)
	{
		//Non-Interrupt modes
		temp = (pGPIOx_Handle->GPIO_PinConfig.GPIO_PinMode)<<(2*pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo);
		pGPIOx_Handle->pGPIOx->MODER &= ~(0x3)<<(2*pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo); //Clearing the required bits
		pGPIOx_Handle->pGPIOx->MODER |= temp;
	}

	//Speed configuration
	temp = (pGPIOx_Handle->GPIO_PinConfig.GPIO_PinSpeed)<<(2*pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo);
	pGPIOx_Handle->pGPIOx->OSPEEDR &= ~(0x3)<<(2*pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo);	  //Clearing the required bits
	pGPIOx_Handle->pGPIOx->OSPEEDR |= temp;

	//Output type configuration
	temp = (pGPIOx_Handle->GPIO_PinConfig.GPIO_PinOPtype)<<(pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo);
	pGPIOx_Handle->pGPIOx->OTYPER &= ~(0x1)<<(pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo);	 //Clearing the required bits
	pGPIOx_Handle->pGPIOx->OTYPER |= temp;

	//Pull-Up/Down configuration
	temp = (pGPIOx_Handle->GPIO_PinConfig.GPIO_PinPuPdCtrl)<<(pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo);
	pGPIOx_Handle->pGPIOx->PUPDR &= ~(0x1)<<(pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo);		//Clearing the required bits
	pGPIOx_Handle->pGPIOx->PUPDR |= temp;

	//Alternate functionality configuration
	temp = (pGPIOx_Handle->GPIO_PinConfig.GPIO_PinAltFunc)<<(4*(pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo%8));
	pGPIOx_Handle->pGPIOx->AF[pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo/8] &= ~(0xF)<<(4*(pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo%8));	//Clearing the required bits
	pGPIOx_Handle->pGPIOx->AF[pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo/8] |= temp;
}

/*
 * 	@function				: GPIO_DeInit
 * 	@info					: De-initializing the GPIO pins
 *
 * 	@param[in]_datatypes	: GPIO_RegDef_t
 * 	@param[in] variables	: GPIO_RegDef_t *pGPIOx
 *
 * 	@return					: void
 *
 * 	@notes					: API for de-initializing the GPIO pins
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	uint8_t temp;

	if(pGPIOx==GPIOA)
		GPIOA_RST();
	else if(pGPIOx==GPIOB)
		GPIOB_RST();
	else if(pGPIOx==GPIOC)
		GPIOC_RST();
	else if(pGPIOx==GPIOD)
		GPIOD_RST();
	else if(pGPIOx==GPIOE)
		GPIOE_RST();
	else if(pGPIOx==GPIOF)
		GPIOF_RST();
	else if(pGPIOx==GPIOG)
		GPIOG_RST();
	else if(pGPIOx==GPIOH)
		GPIOH_RST();
	else if(pGPIOx==GPIOI)
		GPIOI_RST();
}

/*
 * 	@function				: GPIO_IPinRead
 * 	@info					: Read a GPIO pin
 *
 * 	@param[in]_datatypes	: GPIO_RegDef_t, uint8_t
 * 	@param[in] variables	: GPIO_RegDef_t *pGPIOx, uint8_t PinNo
 *
 * 	@return					: uint8_t
 *
 * 	@notes					: API for reading individual GPIO pins
 * 							  Type-cast cos these are 32-bit registers
 */
uint8_t GPIO_IPinRead(GPIO_RegDef_t *pGPIOx, uint8_t PinNo)
{
	uint8_t value = (uint8_t)(((pGPIOx->IDR)>>PinNo)&0x00000001);
	return value;
}

/*
 * 	@function				: GPIO_PortRead
 * 	@info					: Read a GPIO port
 *
 * 	@param[in]_datatypes	: GPIO_RegDef_t
 * 	@param[in] variables	: GPIO_RegDef_t *pGPIOx
 *
 * 	@return					: uint16_t
 *
 * 	@notes					: API for reading a GPIO port
 * 							  Type-cast cos these are 32-bit registers
 */
uint16_t GPIO_PortRead(GPIO_RegDef_t *pGPIOx)
{
	return ((uint16_t)pGPIOx->IDR);
}

/*
 * 	@function				: GPIO_OPinWrite
 * 	@info					: Write to a GPIO pin
 *
 * 	@param[in]_datatypes	: GPIO_RegDef_t, uint8_t, uint16_t
 * 	@param[in] variables	: GPIO_RegDef_t *pGPIOx, uint8_t PinNo, uint8_t value
 * 	@return					: void
 *
 * 	@notes					: API for writing to a GPIO pin
 */
void GPIO_OPinWrite(GPIO_RegDef_t *pGPIOx, uint8_t PinNo, uint8_t value)
{
	if(value==SET)
		pGPIOx->ODR |= (1<<PinNo);
	else if(value==RESET)
		pGPIOx->ODR &= ~(1<<PinNo);
}

/*
 * 	@function				: GPIO_OPortWrite
 * 	@info					: Write to a GPIO port
 *
 * 	@param[in]_datatypes	: GPIO_RegDef_t, uint16_t
 * 	@param[in] variables	: GPIO_RegDef_t *pGPIOx, uint16_t value
 * 	@return					: void
 *
 * 	@notes					: API for writing to a GPIO port
 */
void GPIO_OPortWrite(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}

/*
 * 	@function				: GPIO_ToggleOpin
 * 	@info					: Toggle GPIO output pin
 *
 * 	@param[in]_datatypes	: GPIO_RegDef_t, uint8_t
 * 	@param[in] variables	: GPIO_RegDef_t *pGPIOx, uint8_t PinNo
 * 	@return					: void
 *
 * 	@notes					: API for toggling a GPIO pin
 */
void GPIO_ToggleOpin(GPIO_RegDef_t *pGPIOx, uint8_t PinNo)
{
	(pGPIOx->ODR) ^= (1<<PinNo);
}

void GPIO_IRQconfig(void);
void GPIO_IRQhandler(void);

#endif /* STM32F404XX_GPIO_DRV_H_ */
