/*
 * 003_SPI_SendData_Test1
 *
 *  Created on: Jan 10, 2022
 *      Author: snejDev
 */

#include <stm32f407xx.h>
#include <stm32f407xx_gpio_drv.h>
#include <stm32f407xx_spi_driver.h>

/*AF Mapping
 * 	Alternate Functionality 4
 * 	PB9  --> SPI2_NSS
 * 	PB10 --> SPI2_SCK
 * 	PB14 --> SPI2_MISO
 * 	PB15 --> SPI2_MOSI
 */

//Function Parameters
void GPIO_inits();
void SPI_inits();

void GPIO_inits()
{
	GPIOx_Handle_t gpio;

	GPIO_PCLK_Ctrl(GPIOB, ENABLE);

	gpio.pGPIOx = GPIOB;
	gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF;
	gpio.GPIO_PinConfig.GPIO_PinOPtype = GPIO_PP;
	gpio.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_NPUPD;
	gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_PINSPEED_LOW;
	gpio.GPIO_PinConfig.GPIO_PinAltFunc = 5;

	//NSS
	gpio.GPIO_PinConfig.GPIO_PinNo = GPIO_PINNO_9;
	GPIO_Init(&gpio);

	//SCK
	gpio.GPIO_PinConfig.GPIO_PinNo = GPIO_PINNO_10;
	GPIO_Init(&gpio);

	//MISO
	gpio.GPIO_PinConfig.GPIO_PinNo = GPIO_PINNO_14;
	GPIO_Init(&gpio);

	//MOSI
	gpio.GPIO_PinConfig.GPIO_PinNo = GPIO_PINNO_15;
	GPIO_Init(&gpio);
}

void SPI_inits()
{
	SPIx_Handle_t spi;

	SPI_PCLK_Ctrl(SPI2, ENABLE);

	spi.pSPIx = SPI2;
	spi.SPIx_PinConfig.SPI_DeviceConfig = MASTER;
	spi.SPIx_PinConfig.SPI_BusConfig = BUSCONF_FULLDUP;
	spi.SPIx_PinConfig.SPI_ClkSpeed = BRDIV256;
	spi.SPIx_PinConfig.SPI_CPOL = ZERO_IDLE;
	spi.SPIx_PinConfig.SPI_CPHA = FIRSTCLK_CAP;
	spi.SPIx_PinConfig.SPI_DFF = DFF8;
	spi.SPIx_PinConfig.SPI_SSM = SSM_EN;

	SPI_Init(&spi);
}

int main(void)
{
 	char data[] = "SPI_DRIVER TEST";

	GPIO_inits();
	SPI_inits();

	/*SPE bit is asserted LOW by default and has to be kept that way
	 *till all configuration registers are asserted. Following this, the bit
	 *is asserted, and the comm starts*/
	SPI_SSIConfig(SPI2,ENABLE);
	SPI_EN(SPI2,ENABLE);


	SPI_TxDataB(SPI2,(uint8_t*)data,strlen(data));

	SPI_SSIConfig(SPI2,DISABLE);
	while(1);
	return 0;
}
