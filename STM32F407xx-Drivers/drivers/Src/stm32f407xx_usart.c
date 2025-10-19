/*
 ******************************************************************************
 * @file    stm32f407xx_usart.c
 * @author  Omer Faruk Yesir
 * @brief   This file provides functions for the USART peripheral
 *
 ******************************************************************************
 */

#include "stm32f407xx_usart.h"
#include "stm32f407xx_rcc.h"



/*********************************************************************
 * @fn      		  - USART_PeriClockControl
 *
 * @brief             - Enables or disables peripheral clock for the given USART
 *
 * @param[in]         - Base address of USART peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            - none
 *
 * @Note              - none
 *********************************************************************/
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pUSARTx == USART1)
			USART1_PCLK_EN();
		else if(pUSARTx == USART2)
			USART2_PCLK_EN();
		else if(pUSARTx == USART3)
			USART3_PCLK_EN();
		else if(pUSARTx == UART4)
			UART4_PCLK_EN();
		else if(pUSARTx == UART5)
			UART5_PCLK_EN();
		else if(pUSARTx == USART6)
			USART6_PCLK_EN();
	}
	else
	{
		// TODO: Implement clock disable macros
	}
}



/*********************************************************************
 * @fn      		  - USART_SetBaudRate
 *
 * @brief             - Configures the baud rate for USART
 *
 * @param[in]         - Base address of USART peripheral
 * @param[in]         - Baud rate value
 *
 * @return            - none
 *
 * @Note              - Uses APB clock and calculates USARTDIV
 *********************************************************************/
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
	uint32_t PCLKx;
	uint32_t usartdiv;
	uint32_t M_part, F_part;
	uint32_t tempreg = 0;

	// Get the value of APB bus clock
	if(pUSARTx == USART1 || pUSARTx == USART6)
	{
		// USART1 and USART6 are on APB2
		PCLKx = RCC_GetPCLK2Value();
	}
	else
	{
		// USART2, USART3, UART4, UART5 are on APB1
		PCLKx = RCC_GetPCLK1Value();
	}

	// Check for OVER8 configuration bit
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
		// OVER8 = 1, oversampling by 8
		usartdiv = ((25 * PCLKx) / (2 * BaudRate));
	}
	else
	{
		// OVER8 = 0, oversampling by 16
		usartdiv = ((25 * PCLKx) / (4 * BaudRate));
	}

	// Calculate mantissa part
	M_part = usartdiv / 100;
	tempreg |= (M_part << 4);

	// Calculate fractional part
	F_part = (usartdiv - (M_part * 100));

	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
		// OVER8 = 1, oversampling by 8
		F_part = ((F_part * 8) + 50) / 100;
		F_part &= 0x07;
	}
	else
	{
		// OVER8 = 0, oversampling by 16
		F_part = ((F_part * 16) + 50) / 100;
		F_part &= 0x0F;
	}

	tempreg |= F_part;

	// Copy value to BRR register
	pUSARTx->BRR = tempreg;
}



/*********************************************************************
 * @fn      		  - USART_Init
 *
 * @brief             - Initializes the given USART peripheral
 *
 * @param[in]         - Pointer to USART handle structure
 *
 * @return            - none
 *
 * @Note              - Configures mode, baud rate, word length, stop bits, parity
 *********************************************************************/
void USART_Init(USART_Handle_t *pUSARTHandle)
{
	uint32_t tempreg = 0;

	// Enable USART peripheral clock
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	// 1. Configure CR1 register

	// Enable USART Tx and Rx engines according to mode
	if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		tempreg |= (1 << USART_CR1_RE);
	}
	else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		tempreg |= (1 << USART_CR1_TE);
	}
	else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		tempreg |= ((1 << USART_CR1_TE) | (1 << USART_CR1_RE));
	}

	// Configure word length
	tempreg |= (pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M);

	// Configure parity control
	if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		tempreg |= (1 << USART_CR1_PCE);
		// EVEN parity (PS bit = 0, already cleared)
	}
	else if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD)
	{
		tempreg |= (1 << USART_CR1_PCE);
		tempreg |= (1 << USART_CR1_PS);
	}

	pUSARTHandle->pUSARTx->CR1 = tempreg;

	// 2. Configure CR2 register
	tempreg = 0;

	// Configure stop bits
	tempreg |= (pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP);

	pUSARTHandle->pUSARTx->CR2 = tempreg;

	// 3. Configure CR3 register
	tempreg = 0;

	// Configure hardware flow control
	if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		tempreg |= (1 << USART_CR3_CTSE);
	}
	else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		tempreg |= (1 << USART_CR3_RTSE);
	}
	else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		tempreg |= ((1 << USART_CR3_CTSE) | (1 << USART_CR3_RTSE));
	}

	pUSARTHandle->pUSARTx->CR3 = tempreg;

	// 4. Configure baud rate
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
}



/*********************************************************************
 * @fn      		  - USART_DeInit
 *
 * @brief             - Resets all the USART registers
 *
 * @param[in]         - Pointer to USART handle structure
 *
 * @return            - none
 *
 * @Note              - Uses RCC peripheral reset register
 *********************************************************************/
void USART_DeInit(USART_Handle_t *pUSARTHandle)
{
	if(pUSARTHandle->pUSARTx == USART1)
		USART1_REG_RESET();
	else if(pUSARTHandle->pUSARTx == USART2)
		USART2_REG_RESET();
	else if(pUSARTHandle->pUSARTx == USART3)
		USART3_REG_RESET();
	else if(pUSARTHandle->pUSARTx == UART4)
		UART4_REG_RESET();
	else if(pUSARTHandle->pUSARTx == UART5)
		UART5_REG_RESET();
	else if(pUSARTHandle->pUSARTx == USART6)
		USART6_REG_RESET();
}



/*********************************************************************
 * @fn      		  - USART_GetFlagStatus
 *
 * @brief             - Returns the status of given flag
 *
 * @param[in]         - Base address of USART peripheral
 * @param[in]         - Flag name (macro)
 *
 * @return            - Flag status (SET or RESET)
 *
 * @Note              - Checks SR register
 *********************************************************************/
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName)
{
	if(pUSARTx->SR & StatusFlagName)
	{
		return SET;
	}
	return RESET;
}



/*********************************************************************
 * @fn      		  - USART_ClearFlag
 *
 * @brief             - Clears the given flag
 *
 * @param[in]         - Base address of USART peripheral
 * @param[in]         - Flag name
 *
 * @return            - none
 *
 * @Note              - Some flags are cleared by reading SR then DR
 *********************************************************************/
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{
	pUSARTx->SR &= ~(StatusFlagName);
}



/*********************************************************************
 * @fn      		  - USART_PeripheralControl
 *
 * @brief             - Enables or disables USART peripheral
 *
 * @param[in]         - Base address of USART peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            - none
 *
 * @Note              - Controls UE bit in CR1 register
 *********************************************************************/
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	}
	else
	{
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}
}



/*********************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief             - Sends data over USART (blocking)
 *
 * @param[in]         - Pointer to USART handle structure
 * @param[in]         - Pointer to Tx buffer
 * @param[in]         - Length of data to send
 *
 * @return            - none
 *
 * @Note              - This is a blocking call (polling mode)
 *********************************************************************/
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint16_t *pdata;

	// Loop over until Len number of bytes are transferred
	for(uint32_t i = 0; i < Len; i++)
	{
		// Wait until TXE flag is set
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE));

		// Check word length (8 or 9 bits)
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			// 9-bit data transfer
			pdata = (uint16_t*)pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			// Check parity control
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				// No parity, all 9 bits are user data
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				// Parity bit used, only 8 bits are user data
				pTxBuffer++;
			}
		}
		else
		{
			// 8-bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (uint8_t)0xFF);
			pTxBuffer++;
		}
	}

	// Wait until TC flag is set (transmission complete)
	while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC));
}



/*********************************************************************
 * @fn      		  - USART_ReceiveData
 *
 * @brief             - Receives data over USART (blocking)
 *
 * @param[in]         - Pointer to USART handle structure
 * @param[in]         - Pointer to Rx buffer
 * @param[in]         - Length of data to receive
 *
 * @return            - none
 *
 * @Note              - This is a blocking call (polling mode)
 *********************************************************************/
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	// Loop over until Len number of bytes are received
	for(uint32_t i = 0; i < Len; i++)
	{
		// Wait until RXNE flag is set
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE));

		// Check word length (8 or 9 bits)
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			// 9-bit data reception
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				// No parity, all 9 bits are data
				*((uint16_t*)pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t)0x01FF);
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				// Parity used, only 8 bits are data
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
				pRxBuffer++;
			}
		}
		else
		{
			// 8-bit data reception
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				// No parity, all 8 bits are data
				*pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
			}
			else
			{
				// Parity used, only 7 bits are data
				*pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);
			}
			pRxBuffer++;
		}
	}
}



/*********************************************************************
 * @fn      		  - USART_SendDataIT
 *
 * @brief             - Sends data in interrupt mode
 *
 * @param[in]         - Pointer to USART handle structure
 * @param[in]         - Pointer to Tx buffer
 * @param[in]         - Length of data to send
 *
 * @return            - State (READY or BUSY)
 *
 * @Note              - This is non-blocking call
 *********************************************************************/
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txstate = pUSARTHandle->TxBusyState;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		// Enable interrupt for TXE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);

		// Enable interrupt for TC
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);
	}

	return txstate;
}



/*********************************************************************
 * @fn      		  - USART_ReceiveDataIT
 *
 * @brief             - Receives data in interrupt mode
 *
 * @param[in]         - Pointer to USART handle structure
 * @param[in]         - Pointer to Rx buffer
 * @param[in]         - Length of data to receive
 *
 * @return            - State (READY or BUSY)
 *
 * @Note              - This is non-blocking call
 *********************************************************************/
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if(rxstate != USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		// Enable interrupt for RXNE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
	}

	return rxstate;
}



/*********************************************************************
 * @fn      		  - USART_IRQInterruptConfig
 *
 * @brief             - Configures interrupt
 *
 * @param[in]         - IRQ interrupt number
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            - none
 *
 * @Note              - Configures NVIC registers
 *********************************************************************/
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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



/*********************************************************************
 * @fn      		  - USART_IRQPriorityConfig
 *
 * @brief             - Configures interrupt priority
 *
 * @param[in]         - IRQ interrupt number
 * @param[in]         - IRQ interrupt priority
 *
 * @return            - none
 *
 * @Note              - Configures NVIC priority registers
 *********************************************************************/
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}



/*********************************************************************
 * @fn      		  - USART_IRQHandling
 *
 * @brief             - Handles USART interrupts
 *
 * @param[in]         - Pointer to USART handle structure
 *
 * @return            - none
 *
 * @Note              - Handles TC, TXE, RXNE, and error interrupts
 *********************************************************************/
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{
	uint32_t temp1, temp2, temp3;
	uint16_t *pdata;

	// Check for TC (Transmission Complete) flag
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TC);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TCIE);

	if(temp1 && temp2)
	{
		// Close transmission and call application callback
		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			if(!pUSARTHandle->TxLen)
			{
				// Clear TC flag
				pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_TC);

				// Clear TCIE bit
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TCIE);

				// Reset application state
				pUSARTHandle->TxBusyState = USART_READY;
				pUSARTHandle->pTxBuffer = 0;
				pUSARTHandle->TxLen = 0;

				// Call application callback
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_TX_CMPLT);
			}
		}
	}

	// Check for TXE (Transmit Data Register Empty) flag
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TXE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TXEIE);

	if(temp1 && temp2)
	{
		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			if(pUSARTHandle->TxLen > 0)
			{
				// Check word length
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					pdata = (uint16_t*)pUSARTHandle->pTxBuffer;
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen -= 2;
					}
					else
					{
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen--;
					}
				}
				else
				{
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer & (uint8_t)0xFF);
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxLen--;
				}
			}

			if(pUSARTHandle->TxLen == 0)
			{
				// Disable TXEIE interrupt
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE);
			}
		}
	}

	// Check for RXNE (Receive Data Register Not Empty) flag
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_RXNE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);

	if(temp1 && temp2)
	{
		if(pUSARTHandle->RxBusyState == USART_BUSY_IN_RX)
		{
			if(pUSARTHandle->RxLen > 0)
			{
				// Check word length
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						*((uint16_t*)pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t)0x01FF);
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen -= 2;
					}
					else
					{
						*pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen--;
					}
				}
				else
				{
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						*pUSARTHandle->pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
					}
					else
					{
						*pUSARTHandle->pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);
					}
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->RxLen--;
				}
			}

			if(!pUSARTHandle->RxLen)
			{
				// Disable RXNEIE interrupt
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_RXNEIE);

				// Reset application state
				pUSARTHandle->RxBusyState = USART_READY;

				// Call application callback
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_RX_CMPLT);
			}
		}
	}

	// Check for CTS (Clear To Send) flag
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_CTS);
	temp2 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSE);
	temp3 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSIE);

	if(temp1 && temp2 && temp3)
	{
		// Clear CTS flag
		pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_CTS);

		// Call application callback
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_CTS);
	}

	// Check for IDLE line detection flag
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_IDLE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_IDLEIE);

	if(temp1 && temp2)
	{
		// Clear IDLE flag (read SR then DR)
		temp1 = pUSARTHandle->pUSARTx->SR;
		temp1 = pUSARTHandle->pUSARTx->DR;
		(void)temp1;

		// Call application callback
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_IDLE);
	}

	// Check for Overrun error
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_ORE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);

	if(temp1 && temp2)
	{
		// Clear ORE flag (read SR then DR)
		temp1 = pUSARTHandle->pUSARTx->SR;
		temp1 = pUSARTHandle->pUSARTx->DR;
		(void)temp1;

		// Call application callback
		USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_ORE);
	}

	// Check for Framing error
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_FE);
	temp2 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_EIE);

	if(temp1 && temp2)
	{
		// Clear FE flag (read SR then DR)
		temp1 = pUSARTHandle->pUSARTx->SR;
		temp1 = pUSARTHandle->pUSARTx->DR;
		(void)temp1;

		// Call application callback
		USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_FE);
	}

	// Check for Noise error
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_NE);
	temp2 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_EIE);

	if(temp1 && temp2)
	{
		// Clear NE flag (read SR then DR)
		temp1 = pUSARTHandle->pUSARTx->SR;
		temp1 = pUSARTHandle->pUSARTx->DR;
		(void)temp1;

		// Call application callback
		USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_NE);
	}

	// Check for Parity error
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_PE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_PEIE);

	if(temp1 && temp2)
	{
		// Clear PE flag (read SR then DR)
		temp1 = pUSARTHandle->pUSARTx->SR;
		temp1 = pUSARTHandle->pUSARTx->DR;
		(void)temp1;

		// Call application callback
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_PE);
	}
}



/*********************************************************************
 * @fn      		  - USART_ApplicationEventCallback
 *
 * @brief             - Weak implementation of callback function
 *
 * @param[in]         - Pointer to USART handle structure
 * @param[in]         - Application event
 *
 * @return            - none
 *
 * @Note              - This function should be overridden by application
 *********************************************************************/
__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEv)
{
	// This is a weak implementation
	// Application may override this function
}
