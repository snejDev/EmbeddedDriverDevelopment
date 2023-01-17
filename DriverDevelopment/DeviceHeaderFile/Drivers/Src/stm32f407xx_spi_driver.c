/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Jan 8, 2023
 *      Author: snejDev
 */

#include "stm32f407xx.h"
#include "stm32f407xx_spi_driver.h"
#include <string.h>

uint8_t FlagStatus(SPI_RegDef_t *pSPIx, uint32_t flag);

//API Implementations

/*
 * 	@function				: SPI_PCLK_Ctrl
 * 	@info					: SPI Clock control
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
			SPI1_PCLK_EN();
		else if(pSPIx==SPI2)
			SPI2_PCLK_EN();
		else if(pSPIx==SPI3)
			SPI3_PCLK_EN();
	}
	else if(EN_DI==DISABLE)
	{
		if(pSPIx==SPI1)
			SPI1_PCLK_DI();
		else if(pSPIx==SPI2)
			SPI2_PCLK_DI();
		else if(pSPIx==SPI3)
			SPI3_PCLK_DI();
	}
}

/*
 * 	@function				: SPI_Init
 * 	@info					: Initialize SPI
 *
 * 	@param[in]_datatypes	: SPIx_Handle
 * 	@param[in] variables	: SPIx_Handle *pSPIx_Handle
 *
 * 	@return					: void
 *
 * 	@notes					: API for enabling and disabling the clock
 */

void SPI_Init(SPIx_Handle_t *pSPIx_Handle)
{
	uint32_t temp=0;

	//SPI Device Mode
	temp |= (pSPIx_Handle->SPIx_PinConfig.SPI_DeviceConfig)<<SPI_CR1_MSTR;  //MSTR Bit of SPI_CR1

	//SPI Bus Configurations - shorturl.at/cEIX6
	/*
		BIDIMODE		Full-Duplex-0											Half-Duplex-1(Master and Slave work reciprocally)
		RXONLY			0(Full-Duplex) - Simplex Tx(Disconnect MISO),			BIDIOE:	0(Receive)
						1(Rx Only) - Simplex RX											1(Transmit)
	*/
	uint8_t BusConfig = pSPIx_Handle->SPIx_PinConfig.SPI_BusConfig;
	if(BusConfig == BUSCONF_FULLDUP)
		temp &= ~(1<<SPI_CR1_BIDIMODE);
	else if(BusConfig == BUSCONF_HALFDUP)
		temp |= (1<<SPI_CR1_BIDIMODE);
	else if(BusConfig == BUSCONF_SIMPRX)
	{
		temp &= ~(1<<SPI_CR1_BIDIMODE);
		temp |= (1<<SPI_CR1_RXONLY);
	}
	else if(BusConfig == BUSCONF_SIMPTX)	//Disconnect the MISO line
		temp &= ~(1<<SPI_CR1_BIDIMODE);

	//SPI Clock Speed
	temp |= (pSPIx_Handle->SPIx_PinConfig.SPI_ClkSpeed)<<SPI_CR1_BR;

	//SPI Data Frame Format
	if(pSPIx_Handle->SPIx_PinConfig.SPI_DFF == DFF8)
		temp &= ~(1<<SPI_CR1_DFF);
	else if(pSPIx_Handle->SPIx_PinConfig.SPI_DFF == DFF16)
		temp |= (1<<SPI_CR1_DFF);

	//SPI CPOL
	if(pSPIx_Handle->SPIx_PinConfig.SPI_CPOL == ZERO_IDLE)
		temp &= ~(1<<SPI_CR1_CPOL);
	else if(pSPIx_Handle->SPIx_PinConfig.SPI_CPOL == HIGH_IDLE)
		temp |= (1<<SPI_CR1_CPOL);

	//SPI CPHA
	if(pSPIx_Handle->SPIx_PinConfig.SPI_CPHA == FIRSTCLK_CAP)
		temp &= ~(1<<SPI_CR1_CPHA);
	else if(pSPIx_Handle->SPIx_PinConfig.SPI_CPHA == SECONDCLK_CAP)
		temp |= (1<<SPI_CR1_CPHA);

	//Software Slave Management(SSM)
	if(pSPIx_Handle->SPIx_PinConfig.SPI_SSM == SSM_DI)
		temp &= ~(1<<SPI_CR1_SSM);
	else if(pSPIx_Handle->SPIx_PinConfig.SPI_SSM == SSM_EN)
		temp |= (1<<SPI_CR1_SSM);

	pSPIx_Handle->pSPIx->SPI_CR1 &= ~(temp);
	pSPIx_Handle->pSPIx->SPI_CR1 |= temp;
}

/*
 * 	@function				: SPI_DeInit
 * 	@info					: De-Initialize SPI
 *
 * 	@param[in]_datatypes	: SPI_RegDef_t*
 * 	@param[in] variables	: SPI_RegDef_t *pSPIx
 *
 * 	@return					: void
 *
 * 	@notes					: API for reseting the SPI peripheral
 */

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx==SPI1)
		SPI1_RST();
	else if(pSPIx==SPI2)
		SPI2_RST();
	else if(pSPIx==SPI3)
		SPI3_RST();
}

/*
 * 	@function				: SPI_TxData
 * 	@info					: Transmit Data
 *
 * 	@param[in]_datatypes	: SPI_RegDef_t*, uint8_t*, uint32_t*
 * 	@param[in] variables	: (SPI_RegDef_t *pSPIx, uint8_t *pTxBuff, uint32_t len
 *
 * 	@return					: void
 *
 * 	@notes					: len variable carries the number of data bytes transmit
 * 							  pTxBuff hold a pointer to the start of the data
 * 							  * when length>0, and the TXE bit is set,
 * 							  		DFF_8:	1 byte of data is written to the data register
 * 							  				len decremented by 1, loop back to *
 * 							  		DFF_16: 2 bytes of data is written to the data register
 * 							  				len decremented by 2, loop back to *
 */
void SPI_TxDataB(SPI_RegDef_t *pSPIx, uint8_t *pTxBuff, uint32_t len)
{
	//Tx Data Blocking Call API: Blocks Program execution, till all the data is transmitted
	while(len>0)
	{
		while(!(FlagStatus(pSPIx, SPI_SR_TXEM)));	//wait until TXE bit is SET

		if(pSPIx->SPI_CR1 & (1<<SPI_CR1_DFF))			//16 bit data Tx
		{
			pSPIx->SPI_DR = *((uint16_t*)pTxBuff);		//DR is 16 bits
			(uint16_t*)pTxBuff++;
			len -= 2;
		}
		else	//8 bit data Tx
		{
			pSPIx->SPI_DR = *pTxBuff;		//DR is 8 bits
			pTxBuff++;
			len--;
		}
	}
}

/*
 * 	@function				: SPI_EN
 * 	@info					: Set/Clear SPE bit
 *
 * 	@param[in]_datatypes	: SPI_RegDef_t*, uint8_t*
 * 	@param[in] variables	: SPI_RegDef_t *pSPIx,uint8_t EN_DI
 *
 * 	@return					: void
 *
 * 	@notes					: Enables/Disables SPI by setting the SPE bit
 * 							  Call the function after all the register config
 */
void SPI_EN(SPI_RegDef_t *pSPIx,uint8_t EN_DI)
{
	pSPIx->SPI_CR1 &= ~(1<<SPI_CR1_SPE);
	if(EN_DI==ENABLE)
		(pSPIx->SPI_CR1) |= (1<<SPI_CR1_SPE);
}

/*
 * 	@function				: SPI_SSIConfig
 * 	@info					: Set/Clear SSI bit
 *
 * 	@param[in]_datatypes	: SPI_RegDef_t*, uint8_t*
 * 	@param[in] variables	: SPI_RegDef_t *pSPIx,uint8_t EN_DI
 *
 * 	@return					: void
 *
 * 	@notes					: Enables/Disables SPI by setting the SSI bit
 * 							  Call the function after all the register config
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EN_DI)
{
	if(EN_DI==ENABLE)
		(pSPIx->SPI_CR1) |= (1<<SPI_CR1_SSI);
	else if(EN_DI==DISABLE)
		(pSPIx->SPI_CR1) &= ~(1<<SPI_CR1_SSI);
}

/* 	@function				: SPI_EN
 * 	@info					: Set/Clear SSOE bit
 *
 * 	@param[in]_datatypes	: SPI_RegDef_t*, uint8_t*
 * 	@param[in] variables	: SPI_RegDef_t *pSPIx,uint8_t EN_DI
 *
 * 	@return					: void
 *
 * 	@notes					: Enables the output by setting the SSOE bit of SPI_CR2
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EN_DI)
{
	if(EN_DI==ENABLE)
		(pSPIx->SPI_CR2) |= (1<<SPI_CR2_SSOE);
	else if(EN_DI == DISABLE)
		(pSPIx->SPI_CR2) &= ~(1<<SPI_CR2_SSOE);
}

/*
 * 	@function				: FlagStatus
 * 	@info					: Checks if a certain bit is set/reset in the Status Register
 *
 * 	@param[in]_datatypes	: SPI_RegDef_t*, uint32_t
 * 	@param[in] variables	: SPI_RegDef_t *pSPIx, uint32_t flag
 *
 * 	@return					: void
 *
 * 	@notes					: Checks if a bit
 */

uint8_t FlagStatus(SPI_RegDef_t *pSPIx, uint32_t flag)
{
	if((pSPIx->SPI_SR) & flag)
		return SET;
	return RESET;
}


