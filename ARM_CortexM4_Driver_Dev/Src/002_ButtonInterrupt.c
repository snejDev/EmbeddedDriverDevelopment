/*
 * 002_ButtonInterrupt.c
 *
 *  Created on: Oct 28, 2022
 *      Author: snejdev
 */

#include <stdint.h>
#include <string.h>
#include "stm32f407xx.h"
#include "stm32f407xx_gpio_drv.h"

void GPIO_PinConfig(void);
void EXTI0_IRQHandler(void);
void delay(void);

void GPIO_PinConfig(void)
{
	GPIOx_Handle_t gpio;

	//Set every member of the structure to 0
	memset(&gpio,0,sizeof(gpio));

	GPIO_PCLK_Ctrl(GPIOD, ENABLE);
	gpio.pGPIOx = GPIOD;

	gpio.GPIO_PinConfig.GPIO_PinNo = GPIO_PINNO_2;
	gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio.GPIO_PinConfig.GPIO_PinOPtype = GPIO_PP;
	gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_PINSPEED_HIGH;
	gpio.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_NPUPD;

	GPIO_Init(&gpio);

	memset(&gpio,0,sizeof(gpio));
	gpio.pGPIOx = GPIOD;

	gpio.GPIO_PinConfig.GPIO_PinNo = GPIO_PINNO_0;
	gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_PINSPEED_HIGH;
	gpio.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_NPUPD;

	GPIO_Init(&gpio);
}

int main(void)
{
	GPIO_PinConfig();

	GPIO_OPinWrite(GPIOD, GPIO_PINNO_2, SET);
	//Enable IRQ and configure priority
	GPIO_IRQconfig(IRQ_NO_EXTI0, ENABLE, IRQ_PRI_15);
	//Override ISR
	EXTI0_IRQHandler();
	while(1);
}

//ISR Override
void EXTI0_IRQHandler(void)
{
	GPIO_IRQhandler(GPIO_PINNO_0);
	GPIO_ToggleOpin(GPIOD, GPIO_PINNO_2);
	delay();
}

void delay()
{
	for(int i=0;i<500000;i++);
}
