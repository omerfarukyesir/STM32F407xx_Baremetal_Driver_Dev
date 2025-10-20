/*
 ******************************************************************************
 * @file    stm32f407xx_rcc.c
 * @author  Omer Faruk Yesir
 * @brief   This file provides functions for RCC (Reset and Clock Control)
 *
 ******************************************************************************
 */

#include "stm32f407xx_rcc.h"



/*
 * AHB prescaler values
 */
uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};

/*
 * APB1 prescaler values
 */
uint8_t APB1_PreScaler[4] = {2, 4, 8, 16};

/*
 * APB2 prescaler values
 */
uint8_t APB2_PreScaler[4] = {2, 4, 8, 16};



/*********************************************************************
 * @fn      		  - RCC_GetPCLK1Value
 *
 * @brief             - This function calculates and returns APB1 peripheral clock value
 *
 * @param[in]         - none
 *
 * @return            - APB1 clock value in Hz
 *
 * @Note              - This function considers system clock source (HSI/HSE/PLL)
 *                      and applies AHB and APB1 prescalers
 *********************************************************************/
uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, SystemClk;
	uint8_t clksrc, temp, ahbp, apb1p;

	// Read system clock source from CFGR register (SWS bits)
	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0)
	{
		SystemClk = 16000000;   // HSI oscillator used as system clock
	}
	else if(clksrc == 1)
	{
		SystemClk = 8000000;    // HSE oscillator used as system clock
	}
	else if(clksrc == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();   // PLL used as system clock
	}

	// Read AHB prescaler value (HPRE bits)
	temp = ((RCC->CFGR >> 4) & 0xF);

	if(temp < 8)
	{
		ahbp = 1;   // No division
	}
	else
	{
		ahbp = AHB_PreScaler[temp - 8];
	}

	// Read APB1 prescaler value (PPRE1 bits)
	temp = ((RCC->CFGR >> 10) & 0x7);

	if(temp < 4)
	{
		apb1p = 1;   // No division
	}
	else
	{
		apb1p = APB1_PreScaler[temp - 4];
	}

	// Calculate APB1 clock
	pclk1 = (SystemClk / ahbp) / apb1p;

	return pclk1;
}



/*********************************************************************
 * @fn      		  - RCC_GetPCLK2Value
 *
 * @brief             - This function calculates and returns APB2 peripheral clock value
 *
 * @param[in]         - none
 *
 * @return            - APB2 clock value in Hz
 *
 * @Note              - This function considers system clock source (HSI/HSE/PLL)
 *                      and applies AHB and APB2 prescalers
 *********************************************************************/
uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t pclk2, SystemClk;
	uint8_t clksrc, temp, ahbp, apb2p;

	// Read system clock source from CFGR register (SWS bits)
	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0)
	{
		SystemClk = 16000000;   // HSI oscillator used as system clock
	}
	else if(clksrc == 1)
	{
		SystemClk = 8000000;    // HSE oscillator used as system clock
	}
	else if(clksrc == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();   // PLL used as system clock
	}

	// Read AHB prescaler value (HPRE bits)
	temp = ((RCC->CFGR >> 4) & 0xF);

	if(temp < 8)
	{
		ahbp = 1;   // No division
	}
	else
	{
		ahbp = AHB_PreScaler[temp - 8];
	}

	// Read APB2 prescaler value (PPRE2 bits)
	temp = ((RCC->CFGR >> 13) & 0x7);

	if(temp < 4)
	{
		apb2p = 1;   // No division
	}
	else
	{
		apb2p = APB2_PreScaler[temp - 4];
	}

	// Calculate APB2 clock
	pclk2 = (SystemClk / ahbp) / apb2p;

	return pclk2;
}



/*********************************************************************
 * @fn      		  - RCC_GetPLLOutputClock
 *
 * @brief             - This function calculates and returns PLL output clock value
 *
 * @param[in]         - none
 *
 * @return            - PLL clock value in Hz
 *
 * @Note              - This function is not implemented yet
 *                      User should implement based on their PLL configuration
 *********************************************************************/
uint32_t RCC_GetPLLOutputClock(void)
{
	
	// PLL_VCO = (HSE_VALUE or HSI_VALUE / PLLM) * PLLN
	// PLL_OUTPUT = PLL_VCO / PLLP
	return 0;
}
