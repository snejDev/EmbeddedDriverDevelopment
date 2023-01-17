#include "stm32f407xx.h"
#include "stm32f407xx_gpio_drv.h"

void GPIO_Config();
void delay();

void delay()
{
	for(int i=0;i<500000;i++);
}

void GPIO_Config()
{
	GPIOx_Handle_t gpio;
	gpio.pGPIOx = GPIOD;

	//GPIOD PCLK ON
	GPIO_PCLK_Ctrl(GPIOD,ENABLE);

	/*GPIO PD2*/
	//GPIO_PinConfig_t gpio.GPIO_PinConfig = gpio.GPIO_PinConfig;
	//GPIO Pin Configuration
	gpio.GPIO_PinConfig.GPIO_PinNo = GPIO_PINNO_2;
	gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio.GPIO_PinConfig.GPIO_PinOPtype = GPIO_PP;
	gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_PINSPEED_HIGH;
	gpio.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_NPUPD;

	//Configure
	GPIO_Init(&gpio);

	/*GPIO PD0*/
	//GPIO_PinConfig_t gpio.GPIO_PinConfig = gpio.GPIO_PinConfig;
	//GPIO Pin Configuration
	gpio.GPIO_PinConfig.GPIO_PinNo = GPIO_PINNO_0;
	gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	//gpio.GPIO_PinConfig.GPIO_PinOPtype = GPIO_PP;
	gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_PINSPEED_HIGH;
	gpio.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_NPUPD;

	//Configure
	GPIO_Init(&gpio);
}

int main(void)
{
	GPIO_Config();

	while(1)
	{
		if(GPIO_IPinRead(GPIOD, GPIO_PINNO_0)==(0x01))
			GPIO_OPinWrite(GPIOD, GPIO_PINNO_2, SET);
		else
			GPIO_OPinWrite(GPIOD, GPIO_PINNO_2, RESET);
	}
}
