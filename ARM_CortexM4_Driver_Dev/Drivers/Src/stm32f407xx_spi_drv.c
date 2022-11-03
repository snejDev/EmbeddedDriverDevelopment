/*
 * stm32f407xx_spi_drv.c
 *
 *  Created on: Nov 1, 2022
 *      Author: snejDev
 */

#include "stm32f407xx.h"
#include "stm32f407xx_spi_drv.h"

//	API Implementation
/*
 * 	@function				: SPI_PCLK_Ctrl
 * 	@info					: Peripheral Clock Control
 *
 * 	@param[in]_datatypes	: SPI_RegDef_t, uint8_t
 * 	@param[in] variables	: SPI_RegDef_t *pSPIx, uint8_t EN_DI
 *
 * 	@return					: void
 *
 * 	@notes					: API for enabling and disabling the clock
 */

void SPI_PCLK_Ctrl(SPI_RegDef_t *pSPIx, uint8_t EN_DI)
{
	if(EN_DI==ENABLE)
	{
		if(pSPIx==SPI1)
			SPI1_CLK_EN();
		else if(pSPIx==SPI2)
			SPI2_CLK_EN();
		else if(pSPIx==SPI3)
			SPI3_CLK_EN();
	}
	else if(EN_DI==DISABLE)
	{
		if(EN_DI==ENABLE)
		{
			if(pSPIx==SPI1)
				SPI1_CLK_DI();
			else if(pSPIx==SPI2)
				SPI2_CLK_DI();
			else if(pSPIx==SPI3)
				SPI3_CLK_DI();
		}
	}
}

/*
 * 	@function				: SPI_Init
 * 	@info					: Initialize SPI Paramaters
 *
 * 	@param[in]_datatypes	: SPIx_Handle_t
 * 	@param[in] variables	: SPIx_Handle_t *pSPIx_Handle
 *
 * 	@return					: void
 *
 * 	@notes					: API for initializing SPI
 */
void SPI_Init(SPIx_Handle_t *pSPIx_Handle)
{
	uint32_t temp = 0;

	//Device Modes
	temp |=	(pSPIx_Handle->SPIx_PinConfig.SPI_DeviceMode)<<2;

	//Bus Configurations
	if(pSPIx_Handle->SPIx_PinConfig.SPI_BusConfig == BUSCONF_FULLDUP)
		temp &= ~(1<<BIDIMODE);		//Bi-directional Mode
	else if(pSPIx_Handle->SPIx_PinConfig.SPI_BusConfig == BUSCONF_HALFDUP)
	{
		temp |= (1<<BIDIOE);			//Tx/Rx
		temp |= (1<<BIDIMODE);
	}
	else if(pSPIx_Handle->SPIx_PinConfig.SPI_BusConfig == BUSCONF_SIMP_RX)
	{
		//Enable bidirectional mode
		temp |= ~(1<<BIDIMODE);
		//Assert RX only to force clock output to slave
		temp |= (1<<RXONLY);
	}
	else if(pSPIx_Handle->SPIx_PinConfig.SPI_BusConfig == BUSCONF_SIMP_TX) //Full-Duplex Mode with MISO disconnected
		temp &= ~(1<<BIDIMODE);		//Disconnect MISO line

	//Data Frame Format
	temp |= (pSPIx_Handle->SPIx_PinConfig.SPI_BusConfig << DFF);

	//Clock Phase
	temp |= pSPIx_Handle->SPIx_PinConfig.SPI_CPHA << CPHA;

	//Clock Polarity
	temp |= pSPIx_Handle->SPIx_PinConfig.SPI_CPOL << CPOL;

	//Slave Select Management
	if(pSPIx_Handle->SPIx_PinConfig.SPI_SSM == SSM_HWM)
		temp &= ~(1 << SSM);
	else if(pSPIx_Handle->SPIx_PinConfig.SPI_SSM == SSM_SWM)
		temp |= 1<<SSM;

	//Serial Clock Speed
	temp |= (pSPIx_Handle->SPIx_PinConfig.SPI_SclkSpeed)<<5;
}









