/*
 * stm32f407xx_spi_drv.h
 *
 *  Created on: Oct 31, 2022
 *      Author: a1c2c
 */

#ifndef INC_STM32F407XX_SPI_DRV_H_
#define INC_STM32F407XX_SPI_DRV_H_

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
	SPIx_RegDef_t *pSPIx;
	SPIx_PinConfig_t SPIx_PinConfig;
}SPIx_Handle_t;

#endif /* INC_STM32F407XX_SPI_DRV_H_ */
