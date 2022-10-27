/*
 * stm32f407xx_gpio_drv.c
 *
 *  Created on: Oct 3, 2022
 *      Author: snejdev
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_drv.h"

uint8_t Port_ConfigCode(GPIO_RegDef_t* pGPIOx);

//	API Implementation
/*
 * 	@function				: GPIO_PCLK_Ctrl
 * 	@info					: Peripheral Clock Control
 *
 * 	@param[in]_datatypes	: GPIO_RegDef_t, uint8_t
 * 	@param[in] variables	: GPIO_RegDef_t *pGPIO, uint8_t EN_DI
 *
 * 	@return					: void
 *
 * 	@notes					: API for enabling and disabling the clock
 */

void GPIO_PCLK_Ctrl(GPIO_RegDef_t *pGPIOx, uint8_t EN_DI)
{
	if(EN_DI==ENABLE)
	{
		if(pGPIOx==GPIOA)
			GPIOA_PCLK_EN();
		else if(pGPIOx==GPIOB)
			GPIOB_PCLK_EN();
		else if(pGPIOx==GPIOC)
			GPIOC_PCLK_EN();
		else if(pGPIOx==GPIOD)
			GPIOD_PCLK_EN();
		else if(pGPIOx==GPIOE)
			GPIOE_PCLK_EN();
		else if(pGPIOx==GPIOF)
			GPIOF_PCLK_EN();
		else if(pGPIOx==GPIOG)
			GPIOG_PCLK_EN();
		else if(pGPIOx==GPIOH)
			GPIOH_PCLK_EN();
		else if(pGPIOx==GPIOI)
			GPIOI_PCLK_EN();
	}
	else if(EN_DI==DISABLE)
	{
		if(pGPIOx==GPIOA)
			GPIOA_PCLK_DI();
		else if(pGPIOx==GPIOB)
			GPIOB_PCLK_DI();
		else if(pGPIOx==GPIOC)
			GPIOC_PCLK_DI();
		else if(pGPIOx==GPIOD)
			GPIOD_PCLK_DI();
		else if(pGPIOx==GPIOE)
			GPIOE_PCLK_DI();
		else if(pGPIOx==GPIOF)
			GPIOF_PCLK_DI();
		else if(pGPIOx==GPIOG)
			GPIOG_PCLK_DI();
		else if(pGPIOx==GPIOH)
			GPIOH_PCLK_DI();
		else if(pGPIOx==GPIOI)
			GPIOI_PCLK_DI();
	}
}

/*
 * 	@function				: GPIO_Init
 * 	@info					: Initializing the required GPIO pins
 *
 * 	@param[in]_datatypes	: GPIO_Handle_t
 * 	@param[in] variables	: GPIO_Handle_t *pGPIOx_Handle
 *
 * 	@return					: void
 *
 * 	@notes					: API for initializing the GPIO pins
 */
void GPIO_Init(GPIOx_Handle_t *pGPIOx_Handle)
{
	uint32_t temp;

	//Pin mode configuration
	if(pGPIOx_Handle->GPIO_PinConfig.GPIO_PinMode<=GPIO_MODE_AN)
	{
		//Non-Interrupt modes
		temp = (pGPIOx_Handle->GPIO_PinConfig.GPIO_PinMode)<<(2*(pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo));
		pGPIOx_Handle->pGPIOx->MODER &= ~((0x3)<<(2*(pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo)));
		pGPIOx_Handle->pGPIOx->MODER |= temp;
		/*temp = (pGPIOx_Handle>GPIO_PinConfig.GPIO_PinMode)<<(2*(pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo));
		(pGPIOx_Handle->pGPIOx->MODER) &= ~(0x3)<<(2*(pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo)); //Clearing the required bits
		(pGPIOx_Handle->pGPIOx->MODER) |= temp;*/
	}
	else
	{
		//Interrupt Mode

		/**EXTI Line Configuration**/
		//IT_RT -  Interrupt Rising Edge Trigger
		if(pGPIOx_Handle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RT)
		{
			//Configure RTSR
			EXTI->EXTI_RTSR &= ~(1<<(pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo));
			EXTI->EXTI_RTSR |= 1<<(pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo);
			//Clear FTSR
			EXTI->EXTI_FTSR &=  ~(1<<(pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo));

		}
		//IT_FT -  Interrupt Falling Edge Trigger
		if(pGPIOx_Handle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_FT)
		{
			//Configure FTSR
			EXTI->EXTI_FTSR &= ~(1<<(pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo));
			EXTI->EXTI_FTSR |= 1<<(pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo);
			//Clear RTSR
			EXTI->EXTI_RTSR &=  ~(1<<(pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo));
		}
		//IT_RFT -  Interrupt Rising/Falling Edge Trigger
		if(pGPIOx_Handle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RFT)
		{
			//Configure RTSR
			EXTI->EXTI_RTSR &= ~(1<<(pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo));
			EXTI->EXTI_RTSR |= 1<<(pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo);
			//Configure FTSR
			EXTI->EXTI_FTSR &= ~(1<<(pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo));
			EXTI->EXTI_FTSR |= 1<<(pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo);
		}
		/**EXTI Line Configuration**/

		//Configure GPIO Port selection Register in SYSCFG
		SYSCFG_CLK_EN();																				//Enable SYSCFG Clock
		uint8_t SYSCFG_ARRAY_INDEX = ((pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo)/4);
		uint8_t SYSCFG_BIT_FIELD= ((pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo)%4)*4;
		SYSCFG->SYSCFG_EXTICR[SYSCFG_ARRAY_INDEX] &= ~(Port_ConfigCode(GPIOA))<<(SYSCFG_BIT_FIELD);		//Clear the Bitfield
		SYSCFG->SYSCFG_EXTICR[SYSCFG_ARRAY_INDEX] = Port_ConfigCode(GPIOD)<<(SYSCFG_BIT_FIELD);			//Assert the Port Config Code

		//Configure EXTI Interrupt delivery using IMR Register
		EXTI->EXTI_IMR |= 1<<(pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo);
	}

	//Speed configuration
	temp = (pGPIOx_Handle->GPIO_PinConfig.GPIO_PinSpeed)<<(2*pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo);
	pGPIOx_Handle->pGPIOx->OSPEEDR &= ~((0x3)<<(2*pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo));	  //Clearing the required bits
	pGPIOx_Handle->pGPIOx->OSPEEDR |= temp;

	//Output type configuration
	temp = (pGPIOx_Handle->GPIO_PinConfig.GPIO_PinOPtype)<<(pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo);
	pGPIOx_Handle->pGPIOx->OTYPER &= ~((0x1)<<(pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo));	 //Clearing the required bits
	pGPIOx_Handle->pGPIOx->OTYPER |= temp;

	//Pull-Up/Down configuration
	temp = (pGPIOx_Handle->GPIO_PinConfig.GPIO_PinPuPdCtrl)<<(pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo);
	pGPIOx_Handle->pGPIOx->PUPDR &= ~((0x1)<<(pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo));		//Clearing the required bits
	pGPIOx_Handle->pGPIOx->PUPDR |= temp;

	//Alternate functionality configuration
	temp = (pGPIOx_Handle->GPIO_PinConfig.GPIO_PinAltFunc)<<(4*(pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo%8));
	pGPIOx_Handle->pGPIOx->AF[pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo/8] &= ~(0xF)<<(4*(pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo%8));	//Clearing the required bits
	pGPIOx_Handle->pGPIOx->AF[pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNo/8] |= temp;
}

/*
 * 	@function				: GPIO_DeInit
 * 	@info					: De-initializing the GPIO pins
 *
 * 	@param[in]_datatypes	: GPIO_RegDef_t
 * 	@param[in] variables	: GPIO_RegDef_t *pGPIOx
 *
 * 	@return					: void
 *
 * 	@notes					: API for de-initializing the GPIO pins
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx==GPIOA)
		GPIOA_RST();
	else if(pGPIOx==GPIOB)
		GPIOB_RST();
	else if(pGPIOx==GPIOC)
		GPIOC_RST();
	else if(pGPIOx==GPIOD)
		GPIOD_RST();
	else if(pGPIOx==GPIOE)
		GPIOE_RST();
	else if(pGPIOx==GPIOF)
		GPIOF_RST();
	else if(pGPIOx==GPIOG)
		GPIOG_RST();
	else if(pGPIOx==GPIOH)
		GPIOH_RST();
	else if(pGPIOx==GPIOI)
		GPIOI_RST();
}

/*
 * 	@function				: GPIO_IPinRead
 * 	@info					: Read a GPIO pin
 *
 * 	@param[in]_datatypes	: GPIO_RegDef_t, uint8_t
 * 	@param[in] variables	: GPIO_RegDef_t *pGPIOx, uint8_t PinNo
 *
 * 	@return					: uint8_t
 *
 * 	@notes					: API for reading individual GPIO pins
 * 							  Type-cast cos these are 32-bit registers
 */
uint8_t GPIO_IPinRead(GPIO_RegDef_t *pGPIOx, uint8_t PinNo)
{
	uint8_t value = (uint8_t)(((pGPIOx->IDR)>>PinNo)&0x00000001);
	return value;
}

/*
 * 	@function				: GPIO_PortRead
 * 	@info					: Read a GPIO port
 *
 * 	@param[in]_datatypes	: GPIO_RegDef_t
 * 	@param[in] variables	: GPIO_RegDef_t *pGPIOx
 *
 * 	@return					: uint16_t
 *
 * 	@notes					: API for reading a GPIO port
 * 							  Type-cast cos these are 32-bit registers
 */
uint16_t GPIO_PortRead(GPIO_RegDef_t *pGPIOx)
{
	return ((uint16_t)pGPIOx->IDR);
}

/*
 * 	@function				: GPIO_OPinWrite
 * 	@info					: Write to a GPIO pin
 *
 * 	@param[in]_datatypes	: GPIO_RegDef_t, uint8_t, uint16_t
 * 	@param[in] variables	: GPIO_RegDef_t *pGPIOx, uint8_t PinNo, uint8_t value
 * 	@return					: void
 *
 * 	@notes					: API for writing to a GPIO pin
 */
void GPIO_OPinWrite(GPIO_RegDef_t *pGPIOx, uint8_t PinNo, uint8_t value)
{
	if(value==SET)
		pGPIOx->ODR |= (1<<PinNo);
	else if(value==RESET)
		pGPIOx->ODR &= ~(1<<PinNo);
}

/*
 * 	@function				: GPIO_OPortWrite
 * 	@info					: Write to a GPIO port
 *
 * 	@param[in]_datatypes	: GPIO_RegDef_t, uint16_t
 * 	@param[in] variables	: GPIO_RegDef_t *pGPIOx, uint16_t value
 * 	@return					: void
 *
 * 	@notes					: API for writing to a GPIO port
 */
void GPIO_OPortWrite(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}

/*
 * 	@function				: GPIO_ToggleOpin
 * 	@info					: Toggle GPIO output pin
 *
 * 	@param[in]_datatypes	: GPIO_RegDef_t, uint8_t
 * 	@param[in] variables	: GPIO_RegDef_t *pGPIOx, uint8_t PinNo
 * 	@return					: void
 *
 * 	@notes					: API for toggling a GPIO pin
 */
void GPIO_ToggleOpin(GPIO_RegDef_t *pGPIOx, uint8_t PinNo)
{
	(pGPIOx->ODR) ^= (1<<PinNo);
}

/*
 * 	@function				: GPIO_IRQconfig
 * 	@info					: IRQ Configuration
 *
 * 	@param[in]_datatypes	: uint8_t, uint8_t, uint8_t
 * 	@param[in] variables	: IRQ_Number, EN_DI, IRQ_Priority
 * 	@return					: void
 *
 * 	@notes					: API for Interrupt configuration
 */
void GPIO_IRQconfig(uint8_t IRQ_Number, uint8_t EN_DI, uint32_t IRQ_Priority)
{
	/**Enable/Disable Interrupts on IRQ Numbers**/
	if(EN_DI == ENABLE)
	{
		if(IRQ_Number<=31)
		{
			//Assert bits in ISER0 Register
			*NVIC_ISER0 |= 1<<(IRQ_Number);
		}
		else if(IRQ_Number>31 && IRQ_Number<=63)
		{
			//Assert bits in ISER1 Register
			*NVIC_ISER1 |= 1<<(IRQ_Number%32);
		}
		else if(IRQ_Number>63 && IRQ_Number<95)
		{
			//Assert bits in ISER2 Register
			*NVIC_ISER2 |= 1<<(IRQ_Number%64);
		}
	}
	else if(EN_DI == DISABLE)
	{
		if(IRQ_Number<=31)
		{
			//Assert bits in ICER0 Register
			*NVIC_ICER0 |= 1<<(IRQ_Number);
		}
		else if(IRQ_Number>31 && IRQ_Number<=63)
		{
			//Assert bits in ICER1 Register
			*NVIC_ICER1 |= 1<<(IRQ_Number%32);
		}
		else if(IRQ_Number>63 && IRQ_Number<95)
		{
			//Assert bits in ICER2 Register
			*NVIC_ICER2 |= 1<<(IRQ_Number%64);
		}
	}
	/**Enable/Disable Interrupts on IRQ Numbers**/

	/**Interrupt Priority Configuration**/
	uint8_t iprx = IRQ_Number/4;
	uint8_t shift = ((IRQ_Number%4)*8)+(8-IMPLEMENTED_BITS);		//Implemented bits are not available
	*(NVIC_IPR_BASE+(iprx)) |= (IRQ_Priority<<shift);
	//*(NVIC_IPR_BASE+(iprx*4)) |= (IRQ_Priority<<shift);
	// Note: There is no need for *4, as NVIC_IPR_BASE being a pointer,
	// Every increment is of uint32_t size
	/**Interrupt Priority Configuration**/
}

void GPIO_IRQhandler(uint8_t PinNo)
{
	//Clear the Pending Register
	if((EXTI->EXTI_PR)&(1<<PinNo))		//PR register bit is set when interrupt occurs
	{
		//PR register bit is cleared by asserting 1 to it
		(EXTI->EXTI_PR) |= (1<<PinNo);
	}
}

uint8_t Port_ConfigCode(GPIO_RegDef_t* pGPIOx)
{
	return ((pGPIOx==GPIOA)?0:\
	   	    (pGPIOx==GPIOB)?1:\
			(pGPIOx==GPIOC)?2:\
			(pGPIOx==GPIOD)?3:\
			(pGPIOx==GPIOE)?4:\
			(pGPIOx==GPIOF)?5:\
			(pGPIOx==GPIOG)?6:\
			(pGPIOx==GPIOH)?7:\
			(pGPIOx==GPIOI)?8:0);
}
