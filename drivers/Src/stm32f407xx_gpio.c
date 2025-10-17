/*
 ******************************************************************************
 * @file    stm32f407xx_gpio.c
 * @author  Omer Faruk Yesir
 * @brief   This file provides functions for the GPIO peripheral
 *
 ******************************************************************************
 */

#include "stm32f407xx_gpio.h"

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - Enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - Base address of GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            - none
 *
 * @Note              - none
 *********************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
			GPIOA_PCLK_EN();
		else if(pGPIOx == GPIOB)
			GPIOB_PCLK_EN();
		else if(pGPIOx == GPIOC)
			GPIOC_PCLK_EN();
		else if(pGPIOx == GPIOD)
			GPIOD_PCLK_EN();
		else if(pGPIOx == GPIOE)
			GPIOE_PCLK_EN();
		else if(pGPIOx == GPIOF)
			GPIOF_PCLK_EN();
		else if(pGPIOx == GPIOG)
			GPIOG_PCLK_EN();
		else if(pGPIOx == GPIOH)
			GPIOH_PCLK_EN();
		else if(pGPIOx == GPIOI)
			GPIOI_PCLK_EN();
	}
	else
	{
		if(pGPIOx == GPIOA)
			GPIOA_PCLK_DI();
		else if(pGPIOx == GPIOB)
			GPIOB_PCLK_DI();
		else if(pGPIOx == GPIOC)
			GPIOC_PCLK_DI();
		else if(pGPIOx == GPIOD)
			GPIOD_PCLK_DI();
		else if(pGPIOx == GPIOE)
			GPIOE_PCLK_DI();
		else if(pGPIOx == GPIOF)
			GPIOF_PCLK_DI();
		else if(pGPIOx == GPIOG)
			GPIOG_PCLK_DI();
		else if(pGPIOx == GPIOH)
			GPIOH_PCLK_DI();
		else if(pGPIOx == GPIOI)
			GPIOI_PCLK_DI();
	}
}
/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - Initializes the given GPIO pin
 *
 * @param[in]         - Pointer to GPIO handle structure
 *
 * @return            - none
 *
 * @Note              - none
 *********************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0; // temporary register value

	// 1. Configure mode of GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// Non-interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <<
			   (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 <<
			   (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // clear
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else
	{
		// Interrupt mode configuration
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// Configure FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear RTSR
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// Configure RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear FTSR
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			// Both edges
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// 2. Configure GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

		// 3. Enable EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;

	// 2. Configure speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed <<
		   (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 <<
		   (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;

	// 3. Configure pull-up/pull-down
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl <<
		   (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 <<
		   (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	// 4. Configure output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType <<
		   (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 <<
		   (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;

	// 5. Configure alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |=
			(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}
}



/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - Resets all the GPIO registers
 *
 * @param[in]         - Base address of the GPIO port
 *
 * @return            - none
 *********************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
		GPIOA_REG_RESET();
	else if(pGPIOx == GPIOB)
		GPIOB_REG_RESET();
	else if(pGPIOx == GPIOC)
		GPIOC_REG_RESET();
	else if(pGPIOx == GPIOD)
		GPIOD_REG_RESET();
	else if(pGPIOx == GPIOE)
		GPIOE_REG_RESET();
	else if(pGPIOx == GPIOF)
		GPIOF_REG_RESET();
	else if(pGPIOx == GPIOG)
		GPIOG_REG_RESET();
	else if(pGPIOx == GPIOH)
		GPIOH_REG_RESET();
	else if(pGPIOx == GPIOI)
		GPIOI_REG_RESET();
}



/*
 * Data read and write
 */
/*****************************************************************
 * @fn          - GPIO_ReadFromInputPin
 *
 * @brief       - This function reads value of input pin, on
 *                a specific port
 *
 * @param[in]   - Base address of the GPIO peripheral
 * @param[in]   - Pin number
 *
 * @return      - Content of the input data
 *
 * @Note        - 0 or 1
 *
 *****************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}



/*****************************************************************
 * @fn          - GPIO_ReadFromInputPort
 *
 * @brief       - This function reads value of input port
 *
 * @param[in]   - Base address of the GPIO peripheral
 *
 * @return      - Content of the input data
 *
 * @Note        - None
 *
 *****************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}



/*****************************************************************
 * @fn          - GPIO_WriteToOutputPin
 *
 * @brief       - This function writes value on a specific
 *                output pin
 *
 * @param[in]   - Base address of the GPIO peripheral
 * @param[in]   - Pin number
 * @param[in]   - Value (Set/Reset Macro)
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
		pGPIOx->ODR |= (1 << PinNumber);
	else
		pGPIOx->ODR &= ~(1 << PinNumber);
}



/*****************************************************************
 * @fn          - GPIO_WriteToOutputPort
 *
 * @brief       - This function writes value on a specific
 *                output port
 *
 * @param[in]   - Base address of the GPIO peripheral
 * @param[in]   - Value (Set/Reset Macro)
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}



/*****************************************************************
 * @fn          - GPIO_ToggleOutputPin
 *
 * @brief       - This function toggles specific output pin
 *
 * @param[in]   - Base address of the GPIO peripheral
 * @param[in]   - Pin number
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}



/*
 * IRQ Configuration and ISR handling
 */
/*****************************************************************
 * @fn          - GPIO_IRQInterruptConfig
 *
 * @brief       - This function configures interrupt
 *
 * @param[in]   - IRQ Interrupt number
 * @param[in]   - Macro: Enable/Disable
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
			*NVIC_ISER0 |= (1 << IRQNumber);
		else if(IRQNumber > 31 && IRQNumber < 64)
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		else if(IRQNumber >= 64 && IRQNumber < 96)
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
	}
	else
	{
		if(IRQNumber <= 31)
			*NVIC_ICER0 |= (1 << IRQNumber);
		else if(IRQNumber > 31 && IRQNumber < 64)
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		else if(IRQNumber >= 64 && IRQNumber < 96)
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
	}
}



/*****************************************************************
 * @fn          - GPIO_IRQPriorityConfig
 *
 * @brief       - This function configures interrupt priority
 *
 * @param[in]   - IRQ Interrupt number
 * @param[in]   - IRQ interrupt priority
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}



/*****************************************************************
 * @fn          - GPIO_IRQHandling
 *
 * @brief       - This function handle interrupts
 *
 * @param[in]   - Pin number
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber)
{
	// Clear EXTI pending register corresponding bit
	if(EXTI->PR & (1 << PinNumber))
	{
		// Clear by writing 1
		EXTI->PR |= (1 << PinNumber);
	}
}
