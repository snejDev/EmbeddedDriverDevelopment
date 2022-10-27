/*
 * stm32f404xx_gpio_drv.h
 *
 *  Created on: Oct 22, 2022
 *      Author: snejdev
 */

#ifndef STM32F404XX_GPIO_DRV_H_
#define STM32F404XX_GPIO_DRV_H_

#include <stdint.h>
#include <stddef.h>
#include "stm32f407xx.h"

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

//	GPIO API Configuration structure
typedef struct
{								//Accepted inits
	uint8_t GPIO_PinNo;			//@PinNo
	uint8_t GPIO_PinMode;		//@PinMode
	uint8_t GPIO_PinSpeed;		//@PinSpeed
	uint8_t GPIO_PinPuPdCtrl;	//@PinPuPdCtrl
	uint8_t GPIO_PinOPtype;		//@PinOPtype
	uint8_t GPIO_PinAltFunc;
}GPIO_PinConfig_t;

//	GPIO API Handle structure
typedef struct
{
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIOx_Handle_t;

//API Prototypes
void GPIO_PCLK_Ctrl(GPIO_RegDef_t *pGPIOx, uint8_t EN_DI);
void GPIO_Init(GPIOx_Handle_t *pGPIOx_Handle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);
uint8_t GPIO_IPinRead(GPIO_RegDef_t *pGPIOx, uint8_t PinNo);
uint16_t GPIO_PortRead(GPIO_RegDef_t *pGPIOx);
void GPIO_OPinWrite(GPIO_RegDef_t *pGPIOx, uint8_t PinNo, uint8_t value);
void GPIO_OPortWrite(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOpin(GPIO_RegDef_t *pGPIOx, uint8_t PinNo);
void GPIO_IRQconfig(uint8_t IRQ_Number, uint8_t EN_DI, uint32_t IRQ_Priority);
void GPIO_IRQhandler(uint8_t PinNo);

#endif /* STM32F404XX_GPIO_DRV_H_ */
