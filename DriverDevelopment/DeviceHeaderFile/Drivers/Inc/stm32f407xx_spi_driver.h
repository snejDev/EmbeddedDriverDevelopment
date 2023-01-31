/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Jan 7, 2023
 *      Author: a1c2c
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"
#include <string.h>

//SPI_CR1 Register Bit Fields
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

//SPI_CR2 Register Bit Fields
#define SPI_CR2_SSOE	2

//SPI SR Register Bit Fields
#define SPI_SR_RXNE		0
#define SPI_SR_TXE		1
#define SPI_SR_CHSIDE	2
#define SPI_SR_UDR		3
#define SPI_SR_CRCERR	4
#define SPI_SR_MODF		5
#define SPI_SR_OVR		6
#define SPI_SR_BSY		7
#define SPI_SR_FRE		8

//SPI SR Register Masks
#define SPI_SR_RXNEM	(1<<SPI_SR_RXNE)
#define SPI_SR_TXEM		(1<<SPI_SR_TXE)
#define SPI_SR_CHSIDEM	(1<<SPI_SR_CHSIDE)
#define SPI_SR_UDRM		(1<<SPI_SR_UDR)
#define SPI_SR_CRCERRM	(1<<SPI_SR_CRCERR)
#define SPI_SR_MODFM	(1<<SPI_SR_MODF)
#define SPI_SR_OVRM		(1<<SPI_SR_OVR)
#define SPI_SR_BSYM		(1<<SPI_SR_BSY)
#define SPI_SR_FREM		(1<<SPI_SR_FREM)

//SPI Device Configurations
//@SPI_DeviceConfig
#define SLAVE  0
#define MASTER 1

//SPI Bus Configurations
//@SPI_Bus_Config
#define BUSCONF_FULLDUP	0
#define BUSCONF_HALFDUP 1
#define BUSCONF_SIMPRX	2
#define BUSCONF_SIMPTX	3

//SPI Baud Rate Control
//@SPI_ClkSpeed
#define BRDIV2		0
#define BRDIV4		1
#define BRDIV8		2
#define BRDIV16		3
#define BRDIV32		4
#define BRDIV64		5
#define BRDIV128	6
#define BRDIV256	7

//SPI CPOL
//@SPI_CPOL
#define ZERO_IDLE	0
#define HIGH_IDLE	1

//SPI CPHA
#define FIRSTCLK_CAP	0
#define SECONDCLK_CAP	1

//SPI Data Frame Format
//@SPI_DFF
#define DFF8	0
#define DFF16	1

//SPI Software Slave Management
//@SPI_SSM
#define SSM_DI	0
#define SSM_EN	1

/**Function Macros to RESET SPI pins**/
#define SPI1_RST()		do{RCCCLK->APB2RSTR |= 1<<12; RCCCLK->APB2RSTR &= ~(1<<12);}while(0)
#define SPI2_RST()		do{RCCCLK->APB1RSTR |= 1<<14; RCCCLK->APB2RSTR &= ~(1<<14);}while(0)
#define SPI3_RST()		do{RCCCLK->APB1RSTR |= 1<<15; RCCCLK->APB2RSTR &= ~(1<<15);}while(0)
/**Function Macros to RESET SPI pins**/

typedef struct
{
	uint8_t SPI_DeviceConfig;	//Master/Slave
	uint8_t SPI_BusConfig;		//Full-Duplex/Half-Duplex/Simplex
	uint8_t SPI_ClkSpeed;		//SPI Clock Speed
	uint8_t SPI_DFF;			//SPI Data Frame Format
	uint8_t SPI_CPOL;			//Clock Polarity
	uint8_t SPI_CPHA;			//Clock Phase
	uint8_t SPI_SSM;			//Slave Select Management
}SPIx_Config_t;

typedef struct
{
	SPI_RegDef_t *pSPIx;			//Store the base address of SPIx Register
	SPIx_Config_t SPIx_PinConfig;	//SPI Pin configuration
}SPIx_Handle_t;

//API Prototypes
void SPI_PCLK_Ctrl(SPI_RegDef_t *pSPIx, uint8_t EN_DI);
void SPI_Init(SPIx_Handle_t *pSPIx_Handle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);
void SPI_TxDataB(SPI_RegDef_t *pSPIx, uint8_t *pTxBuff, uint32_t len);
void SPI_RxDataB(SPI_RegDef_t *pSPIx, uint8_t *pRxBuff, uint32_t len);
void SPI_EN(SPI_RegDef_t *pSPIx,uint8_t EN_DI);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EN_DI);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EN_DI);
void SPI_IRQconfig(uint8_t IRQ_Numberk, uint8_t EN_DI, uint32_t IRQ_Priority);
uint8_t FlagStatus(SPI_RegDef_t *pSPIx, uint32_t flag);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
