/*
 * 005_SPI_CmdTxRx.c
 *
 *  Created on: Jan 19, 2023
 *      Author: snejDev
 */


#include "stm32f407xx.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_gpio_drv.h"
#include <stdbool.h>

/* Program Description: This program enables Master Slave Communication between STM32 and Arduino
 * STM32 - MASTER; ARDUINO UNO - SLAVE
 *
 * SPI Configuration
 * Mode: Full-Duplex
 * DFF: 8 Bit
 * Slave Management: Hardware Slave Management
 * SCLK Speed: 2MHz
 *
 * Functional Description:
 * Master Sends a command: Slave acknowledges with a ACK byte(0xF5)/NACK byte(0xA5)
 *
 * If ACK: Start Transmission based on commands
 * If NACK: End Transmission and display error message
 *
 *Command Definitions: Command Syntax: <cmd_code>	<arg1> <arg2>
 * 	1) cmd_led_ctrl	<pin_no>  <value>
 * 	   <pin_no>	: Digital pin of Arduino UNO(0 to 9)
 * 	   <value>	: ON(1),OFF(0)
 * 	   Slave Action : Control the digital pin(on/off)
 * 	   Slave Return	: void
 *
 * 	2) cmd_sensor_read	<analog_pin_no>
 * 	   <analog_pin_no>	: Analog Pin of Arduino
 * 	   Slave Action	: Slave should read the analog value of the supplied pin
 * 	   Slave Return : 1 byte of analog read value
 *
 * 	3) cmd_led_read	<pin_no>
 * 	   <pin_no>	: Digital pin of the slave, where the LED is connected to
 * 	   Slave Action : Slave should read the status of the LED
 * 	   Slave Return : 1 byte of status reading(ON/OFF)
 *
 * 	4) cmd_print	<len>  <message>
 * 	   <len>	: Length of the text being sent
 * 	   Slave Action	: Print the sent data
 * 	   Slave Return	: void
 *
 * 	5) cmd_id_read
 * 	   Slave Action	: Return the slave ID
 * 	   Slave Return	: 10 bytes of slave ID string
 *
 *Application Workflow: Start -> First Button Press		: cmd_led_ctrl
 * 								 Second Button Press	: cmd_sensor_read
 * 								 Third Button Press		: cmd_led_read
 * 								 Fourth Button Press	: cmd_print
 * 								 Fifth Button Press		: cmd_id_read		-> Start
 *
 */

void GPIO_Config();
void SPI_Config();
void SPI_Ack();
void SPI_Comm();
void delay();

void delay()
{
	for(int i=0;i<500000;i++);
}

void GPIO_Config()
{
	GPIOx_Handle_t gpio;
	gpio.pGPIOx = GPIOD;

	/*
	 * PD2	:	LED
	 * PD0	: 	Button
	 *
	 * PB9	:	NSS
	 * PB10	: 	SCK
	 * PB14	:	MISO
	 * PB15	: 	MOSI
	 */

	//GPIOD PCLK ON
	GPIO_PCLK_Ctrl(GPIOD,ENABLE);
	GPIO_PCLK_Ctrl(GPIOB,ENABLE);

	/*GPIO PD2*/
	gpio.GPIO_PinConfig.GPIO_PinNo = GPIO_PINNO_2;
	gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio.GPIO_PinConfig.GPIO_PinOPtype = GPIO_PP;
	gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_PINSPEED_HIGH;
	gpio.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_NPUPD;
	GPIO_Init(&gpio);

	/*GPIO PD0*/
	gpio.GPIO_PinConfig.GPIO_PinNo = GPIO_PINNO_0;
	gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
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

void SPI_Ack()
{
	uint8_t a = 'y';

	SPI_SSOEConfig(SPI2,ENABLE);

	SPI_EN(SPI2,ENABLE);
	SPI_TxDataB(SPI2,&a,1);
	while(FlagStatus(SPI2,SPI_SR_BSYM));
	SPI_EN(SPI2,DISABLE);

	uint8_t rcvd;
	GPIO_OPinWrite(GPIOD, GPIO_PINNO_2, RESET);
	SPI_EN(SPI2,ENABLE);
	SPI_RxDataB(SPI2,&rcvd,1);
	while(FlagStatus(SPI2,SPI_SR_BSYM));
	SPI_EN(SPI2,DISABLE);

	if(rcvd=='a')										//Turn on LED if a is received
		GPIO_OPinWrite(GPIOD, GPIO_PINNO_2, SET);
	else
		GPIO_OPinWrite(GPIOD, GPIO_PINNO_2, RESET);
}

void SPI_Comm()
{
	int i=0;
	GPIO_OPinWrite(GPIOD, GPIO_PINNO_2, RESET);
	while(i!=1)
	{
		if(i==0)
		{
			GPIO_OPinWrite(GPIOD, GPIO_PINNO_2, RESET);
			while(!(GPIO_IPinRead(GPIOD, GPIO_PINNO_0)))
				GPIO_OPinWrite(GPIOB, GPIO_PINNO_2, RESET);
			GPIO_OPinWrite(GPIOD, GPIO_PINNO_2, SET);
			delay();
			SPI_Ack();
			i++;
		}
	}
}

int main(void)
{
	//Configure SPI and GPIO
	GPIO_Config();
	SPI_Config();

	SPI_Comm();

	return 0;
}
