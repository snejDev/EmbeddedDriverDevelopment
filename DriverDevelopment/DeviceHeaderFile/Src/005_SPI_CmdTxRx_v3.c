/*
 * 005_SPI_CmdTxRx.c
 *
 *  Created on: Jan 19, 2023
 *      Author: snejDev

 * Program Description: This program enables Master Slave Communication between STM32 and Arduino
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
 * Using Semi-hosting to debug and get printfs working
 * 1. Linker argument settings: -specs-rdimon.specs -lc -lrdimon
 * 2. Debug configuration of your application: monitor arm semishoting enable
 * 3: Include the following codes in main.c: extern void initialise _monitor_handles();
 * 											 initialise_monitor_handles()
 *
 */

#include "stm32f407xx.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_gpio_drv.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

//Command Definitions
#define CMD_LED_CTRL		0x01
#define CMD_SENSOR_READ		0x02
#define CMD_LED_READ		0x03
#define CMD_PRINT			0x04
#define CMD_ID_READ			0x05

//CMD_LED_CTRL
#define DPin	2
#define value	SET

//Arduino Analog Pin Definitions
#define ANALOG_PIN0		0
#define ANALOG_PIN1		1
#define ANALOG_PIN2		2
#define ANALOG_PIN3		3
#define ANALOG_PIN4		4

void GPIO_Config();
void SPI_Config();
void SPI_Comm();
void delay();
bool AckVerify(uint8_t ACK_BYTE);

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

/*
 * 	@function				: ButtonResp
 * 	@info					: Deals with the button i/p
 *
 * 	@param[in]_datatypes	: NULL
 * 	@param[in] variables	: NULL
 *
 * 	@return					: void
 *
 * 	@notes					: The function deals with the button response
 */
void ButtonResp()
{
	GPIO_OPinWrite(GPIOD, GPIO_PINNO_2, RESET);
	while(!(GPIO_IPinRead(GPIOD, GPIO_PINNO_0)))
		GPIO_OPinWrite(GPIOD, GPIO_PINNO_2, RESET);
	GPIO_OPinWrite(GPIOD, GPIO_PINNO_2, SET);
	delay();
	GPIO_OPinWrite(GPIOD, GPIO_PINNO_2, RESET);
}

bool AckVerify(uint8_t ACK_BYTE)
{
	if(ACK_BYTE==245)
		return true;
	return false;
}

void SPI_Comm()
{
	int i=0;
	SPI_SSOEConfig(SPI2, ENABLE);		//SSOE : LOW

	for(i=0;i<5;i++)
	{
		uint8_t CMD_CODE;			//To store the command code
		uint8_t ACK_BYTE;			//To store the receieved acknowledgement
		uint8_t DummyByte = 0xFF;	//Dummy byte to initialize SPI Slave Transfer
		uint8_t DummyRead;
		uint8_t args[2];

		ButtonResp();

		//SPI Tx/Rx Configurations
		SPI_EN(SPI2, ENABLE);

		if(i==0)
		{
			args[0] = DPin;
			args[1] = value;

			CMD_CODE = CMD_LED_CTRL;
			SPI_TxDataB(SPI2,&CMD_CODE,1);					//CMD_LED_CTRL
			//Dummy Read to clear RXNE: Clear Rx Buffer
			SPI_RxDataB(SPI2,&DummyRead,1);
			//SPI Slave doesn't initiate Tx, thereby send dummy byte to shift from SR of Slave to SR of Master
			//Sending 8Bit Dummy value, cause SPI is configured with DFF = 8 bits
			SPI_TxDataB(SPI2, &DummyByte,1);
			SPI_RxDataB(SPI2,&ACK_BYTE,1);
			if(!(AckVerify(ACK_BYTE)))	//enter if-block, if AckVerify returns false
				break;

			//Functionality Definition
			SPI_TxDataB(SPI2,args,2);				//Digital Pin
		}

		else if(i==1)
		{
			uint8_t analogRead;

			CMD_CODE = CMD_SENSOR_READ;
			SPI_TxDataB(SPI2,&CMD_CODE,1);

			SPI_RxDataB(SPI2,&DummyRead,1);
			SPI_TxDataB(SPI2,&DummyByte,1);
			SPI_RxDataB(SPI2,&ACK_BYTE,1);
			if(!(AckVerify(ACK_BYTE)))			//enter if-block, if AckVerify returns false
				break;

			//Functionality Definition
			args[0] = ANALOG_PIN0;
			SPI_TxDataB(SPI2,args,1);
			SPI_RxDataB(SPI2,&DummyRead,1);		//Clear RXNE bit
			delay();
			SPI_TxDataB(SPI2,&DummyByte,1);
			SPI_RxDataB(SPI2,&analogRead,1);
			printf("Analog Value: %d\n",analogRead);
			GPIO_OPinWrite(GPIOD, GPIO_PINNO_2, SET);
			delay();
			delay();
		}

		else if(i==2)
		{
			uint8_t ledRead;

			CMD_CODE = CMD_LED_READ;
			SPI_TxDataB(SPI2,&CMD_CODE,1);

			SPI_RxDataB(SPI2,&DummyRead,1);
			SPI_TxDataB(SPI2,&DummyByte,1);
			SPI_RxDataB(SPI2,&ACK_BYTE,1);
			if(!(AckVerify(ACK_BYTE)))			//enter if-block, if AckVerify returns false
				break;

			//Functionality Definition
			args[0] = ANALOG_PIN0;
			SPI_TxDataB(SPI2,args,1);
			SPI_RxDataB(SPI2,&DummyRead,1);		//Clear RXNE bit
			delay();
			SPI_TxDataB(SPI2,&DummyByte,1);
			SPI_RxDataB(SPI2,&ledRead,1);
			printf("LED Status: %d\n",ledRead);
			GPIO_OPinWrite(GPIOD, GPIO_PINNO_2, SET);
			delay();
			delay();
		}

		else if(i==3)
		{
			char data[] = "Testing CMD_Print Command";
			uint8_t len = strlen(data);

			CMD_CODE = CMD_PRINT;
			SPI_TxDataB(SPI2,&CMD_CODE,1);

			SPI_RxDataB(SPI2,&DummyRead,1);
			SPI_TxDataB(SPI2,&DummyByte,1);
			SPI_RxDataB(SPI2,&ACK_BYTE,1);
			if(!(AckVerify(ACK_BYTE)))			//enter if-block, if AckVerify returns false
				break;

			//Functionality Definition
			SPI_TxDataB(SPI2,&len,1);
			SPI_TxDataB(SPI2,(uint8_t*)data,len);
			SPI_RxDataB(SPI2,&DummyRead,1);
		}

		else if(i==4)
		{
			uint8_t SlaveID[11];

			CMD_CODE = CMD_ID_READ;
			SPI_TxDataB(SPI2,&CMD_CODE,1);

			SPI_RxDataB(SPI2,&DummyRead,1);
			SPI_TxDataB(SPI2,&DummyByte,1);
			SPI_RxDataB(SPI2,&ACK_BYTE,1);
			if(!(AckVerify(ACK_BYTE)))			//enter if-block, if AckVerify returns false
				break;

			//printf("Acknowledged\n");
			//Functionality Description
			for(int i=0;i<10;i++)
			{
				SPI_TxDataB(SPI2,&DummyByte,1);
				SPI_RxDataB(SPI2,(SlaveID+i),1);
				//printf("%d\n",i);
			}
			delay();
			SlaveID[10] = '\0';
			printf("%s\n",SlaveID);
			/*if(strcmp(SlaveID,"SlaveID_1")==0)
				GPIO_OPinWrite(GPIOD, GPIO_PINNO_2, SET);*/
		}
		while(FlagStatus(SPI2, SPI_SR_BSYM));
		SPI_EN(SPI2, DISABLE);
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
