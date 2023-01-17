/*
 * stm32f407xx_spi_drv.c
 *
 *  Created on: Nov 1, 2022
 *      Author: snejDev
 */

#include "stm32f407xx.h"
#include "stm32f407xx_spi_drv.h"

uint8_t FlagStatus(SPI_RegDef_t *pGPIOx, uint32_t flag);

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
		temp &= ~(1<<SPI_CR1_BIDIMODE);		//2-line bi-directional Mode - BIDI mode is cleared
	else if(pSPIx_Handle->SPIx_PinConfig.SPI_BusConfig == BUSCONF_HALFDUP)
	{
		//Handle BIDIOE in the application
		//temp |= (1<<BIDIOE);		//Tx/Rx - OUTPUT EN in Bi-directional mode
		temp |= (1<<SPI_CR1_BIDIMODE);		//1-line bi-directional - BIDI mode is set
	}
	else if(pSPIx_Handle->SPIx_PinConfig.SPI_BusConfig == BUSCONF_SIMP_RX)
	{
		//Enable 2-line uni-directional mode -  BIDI mode is cleared
		temp |= ~(1<<SPI_CR1_BIDIMODE);
		//Set RX only to force clock output to slave
		temp |= (1<<SPI_CR1_RXONLY);
	}
	else if(pSPIx_Handle->SPIx_PinConfig.SPI_BusConfig == BUSCONF_SIMP_TX) //Full-Duplex Mode with MISO disconnected
		temp &= ~(1<<SPI_CR1_BIDIMODE);		//2line bi-directional - Disconnect MISO line

	//Data Frame Format
	temp |= (pSPIx_Handle->SPIx_PinConfig.SPI_BusConfig << SPI_CR1_DFF);

	//Clock Phase
	temp |= pSPIx_Handle->SPIx_PinConfig.SPI_CPHA << SPI_CR1_CPHA;

	//Clock Polarity
	temp |= pSPIx_Handle->SPIx_PinConfig.SPI_CPOL << SPI_CR1_CPOL;

	//Slave Select Management
	if(pSPIx_Handle->SPIx_PinConfig.SPI_SSM == SSM_HWM)
		temp &= ~(1 << SPI_CR1_SSM);		//SPI_CR1_SSM bit disabled
	else if(pSPIx_Handle->SPIx_PinConfig.SPI_SSM == SSM_SWM)
		temp |= 1<<SPI_CR1_SSM;				//SPI_CR1_SSM bit enabled

	//Serial Clock Speed
	temp |= (pSPIx_Handle->SPIx_PinConfig.SPI_SclkSpeed)<<SPI_CR1_BR;

	pSPIx_Handle->pSPIx->SPI_CR1 &= ~(temp);
	pSPIx_Handle->pSPIx->SPI_CR1 |= temp;
}

/*
 * 	@function				: SPI_TxData
 * 	@info					: SPI Transmit data
 *
 * 	@param[in]_datatypes	: SPI_RegDef_t, uint8_t, uint32_t
 * 	@param[in] variables	: SPI_RegDef_t *pSPIx, uint8_t *pTxBuff, uint32_t len
 *
 * 	@return					: void
 *
 * 	@notes					: API for transmitting SPI data
 */
void SPI_TxData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuff, uint32_t len)
{
	//Tx Data Blocking Call API -  Blocks program execution till all data is transmitted
	while(len>0)
	{
		while(!(FlagStatus(pSPIx, SPI_SR_TXEM)));		//Wait till TXE is set - Tx buffer is empty

		if(pSPIx->SPI_CR1 & (1<<SPI_CR1_DFF))
		{
			pSPIx->SPI_DR = *((uint16_t*)pTxBuff);		//Load Data Register
			(uint16_t*)pTxBuff++;						//Increment pointer by 16 bits
			len-=2;
		}
		else
		{
			pSPIx->SPI_DR = *((uint8_t*)pTxBuff);		//Load Data Register
			(uint8_t*)pTxBuff++;						//Increment pointer by 8 bits
			len--;
		}
	}
}

uint8_t FlagStatus(SPI_RegDef_t *pSPIx, uint32_t flag)
{
	if(pSPIx->SPI_SR & flag)
		return SET;
	return RESET;
}

















