#include "stm32f407xx.h"
#include "stm32f407xx_gpio_drv.h"
#include "stm32f407xx_spi_driver.h"

/*AF Mapping
 * 	Alternate Functionality 4
 * 	PB9  --> SPI2_NSS
 * 	PB10 --> SPI2_SCK
 * 	PB14 --> SPI2_MISO
 * 	PB15 --> SPI2_MOSI
 */

void GPIO_Config();
void SPI_Config();
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
	GPIO_PCLK_Ctrl(GPIOB, ENABLE);

	/*GPIO PD2*/
	//GPIO_PinConfig_t gpio.GPIO_PinConfig = gpio.GPIO_PinConfig;
	//GPIO Pin Configuration
	gpio.GPIO_PinConfig.GPIO_PinNo = GPIO_PINNO_2;
	gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio.GPIO_PinConfig.GPIO_PinOPtype = GPIO_PP;
	gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_PINSPEED_HIGH;
	gpio.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_NPUPD;
	GPIO_Init(&gpio);

	/*GPIO PD0*/
	//GPIO_PinConfig_t gpio.GPIO_PinConfig = gpio.GPIO_PinConfig;
	//GPIO Pin Configuration
	gpio.GPIO_PinConfig.GPIO_PinNo = GPIO_PINNO_0;
	gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	//gpio.GPIO_PinConfig.GPIO_PinOPtype = GPIO_PP;
	gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_PINSPEED_HIGH;
	gpio.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_NPUPD;
	GPIO_Init(&gpio);

	//GPIOB Pin Config Tx Comm
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

	//Configure
	GPIO_Init(&gpio);
}

void SPI_Config()
{
	SPIx_Handle_t spi;

	SPI_PCLK_Ctrl(SPI2, ENABLE);

	spi.pSPIx = SPI2;
	spi.SPIx_PinConfig.SPI_DeviceConfig = MASTER;
	spi.SPIx_PinConfig.SPI_BusConfig = BUSCONF_FULLDUP;
	spi.SPIx_PinConfig.SPI_ClkSpeed = BRDIV8;				//2MHz
	spi.SPIx_PinConfig.SPI_CPOL = ZERO_IDLE;
	spi.SPIx_PinConfig.SPI_CPHA = FIRSTCLK_CAP;
	spi.SPIx_PinConfig.SPI_DFF = DFF8;
	spi.SPIx_PinConfig.SPI_SSM = SSM_DI;					//Hardware management

	SPI_Init(&spi);
}

int main(void)
{
	GPIO_Config();
	SPI_Config();

	SPI_SSOEConfig(SPI2, ENABLE);	//SSOE : LOW

	char data[] = "Testing Long String Serial Peripheral Interface Transmission";

	GPIO_OPinWrite(GPIOD, GPIO_PINNO_2, RESET);

	while(1)
	{
		while(!(GPIO_IPinRead(GPIOD, GPIO_PINNO_0)))	//Wait till PIN0 Turns HIGH
			GPIO_OPinWrite(GPIOD, GPIO_PINNO_2, RESET);

		GPIO_OPinWrite(GPIOD, GPIO_PINNO_2, SET);
		delay();

		SPI_EN(SPI2, ENABLE);							//SPE=1 : NSS pulled LOW

		uint8_t dataLen = strlen(data);					//calculate data length
		SPI_TxDataB(SPI2, &dataLen, 1);					//Tx Data length, passing data to a pointer *pTxBuff

		SPI_TxDataB(SPI2,(uint8_t*)data,dataLen);	//Tx actual Data, passing data to a pointer *pTxBuff

		while(FlagStatus(SPI2, SPI_SR_BSYM));

		SPI_EN(SPI2, DISABLE);
	}
	return 0;
}
