/*
 * stm32f407xx_spi_drv.h
 *
 *  Created on: Oct 31, 2022
 *      Author: snejDev
 */

#ifndef INC_STM32F407XX_SPI_DRV_H_
#define INC_STM32F407XX_SPI_DRV_H_

#include "stm32f407xx.h"

//SPI_CR1 Register Bit-fields
#define CPHA		0
#define CPOL 		1
#define SSM 		9
#define RXONLY		10
#define DFF			11
#define BIDIOE		14
#define BIDIMODE	15

//Device Modes
//@SPI_DeviceMode
#define MASTER	1
#define SLAVE	0

//Bus Configurations
//@SPI_BusConfig
#define BUSCONF_FULLDUP		0
#define BUSCONF_HALFDUP		1
#define	BUSCONF_SIMP_RX		2
#define	BUSCONF_SIMP_TX		3

//Data Frame Format
//@SPI_DFF
#define BITFRAME_8		0
#define BITFRAME_16		1

//Clock Phase
//@SPI_CPHA
#define FIRSTCLK_CAP	0
#define SECONDCLK_CAP	1

//Clock Polarity
//@SPI_CPOL
#define ZERO_IDLE	0
#define HIGH_IDLE 	1

//Slave Select Management
//@SPI_SSM
#define SSM_HWM		0
#define SSM_SWM		1

//Serial Clock Baud Rate
//@SPI_SclkSpeed
#define BRC_DIV2	0
#define BRC_DIV4	1
#define BRC_DIV8	2
#define BRC_DIV16	3
#define BRC_DIV32	4
#define BRC_DIV64	5
#define BRC_DIV128	6
#define BRC_DIV256	7

typedef struct
{
	uint8_t SPI_DeviceMode;		//Master/Slave
	uint8_t SPI_BusConfig;		//Full-Duplex/Half-Duplex/Simplex
	uint8_t SPI_DFF;			//Data Frame Format
	uint8_t SPI_CPHA;			//Clock Phase
	uint8_t SPI_CPOL;			//Clock Polarity
	uint8_t SPI_SSM;			//Slave Select Management
	uint8_t SPI_SclkSpeed;		//Serial Clock Speed
}SPIx_PinConfig_t;

typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPIx_PinConfig_t SPIx_PinConfig;
}SPIx_Handle_t;

//API Prototypes
void SPI_PCLK_Ctrl(SPI_RegDef_t *pSPIx, uint8_t EN_DI);
void SPI_Init(SPIx_Handle_t *pSPIx_Handle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);
void SPI_TxData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuff, uint32_t len);
void SPI_RxData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuff, uint32_t len);

#endif /* INC_STM32F407XX_SPI_DRV_H_ */
