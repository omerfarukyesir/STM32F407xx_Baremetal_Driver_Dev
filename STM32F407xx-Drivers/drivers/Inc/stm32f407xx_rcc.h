/*
 ******************************************************************************
 * @file    stm32f407xx_rcc.h
 * @author  Omer Faruk Yesir
 * @brief   Header file for RCC driver (Reset and Clock Control)
 *
 ******************************************************************************
 */

#ifndef INC_STM32F407XX_RCC_H_
#define INC_STM32F407XX_RCC_H_

#include "stm32f407xx.h"



/*********************************************************************
 * 						API FUNCTION PROTOTYPES
 *********************************************************************/

/* Get APB1 peripheral clock value */
uint32_t RCC_GetPCLK1Value(void);

/* Get APB2 peripheral clock value */
uint32_t RCC_GetPCLK2Value(void);

/* Get PLL output clock value */
uint32_t RCC_GetPLLOutputClock(void);



#endif /* INC_STM32F407XX_RCC_H_ */
