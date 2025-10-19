/*
 ******************************************************************************
 * @file    stm32f407xx_i2c.c
 * @author  Omer Faruk Yesir
 * @brief   This file provides functions for the I2C peripheral
 *
 ******************************************************************************
 */

#include "stm32f407xx_i2c.h"
#include "stm32f407xx_rcc.h"



/*
 * Private function prototypes
 */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);



/*********************************************************************
 * @fn      		  - I2C_GenerateStartCondition
 *
 * @brief             - This function generates START condition on I2C bus
 *
 * @param[in]         - Base address of I2C peripheral
 *
 * @return            - none
 *
 * @Note              - This is a private helper function
 *********************************************************************/
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}



/*********************************************************************
 * @fn      		  - I2C_ExecuteAddressPhaseWrite
 *
 * @brief             - This function sends slave address with write bit
 *
 * @param[in]         - Base address of I2C peripheral
 * @param[in]         - Slave address (7-bit)
 *
 * @return            - none
 *
 * @Note              - R/W bit is set to 0 for write operation
 *********************************************************************/
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1);   // Clear R/W bit (write mode)
	pI2Cx->DR = SlaveAddr;
}



/*********************************************************************
 * @fn      		  - I2C_ExecuteAddressPhaseRead
 *
 * @brief             - This function sends slave address with read bit
 *
 * @param[in]         - Base address of I2C peripheral
 * @param[in]         - Slave address (7-bit)
 *
 * @return            - none
 *
 * @Note              - R/W bit is set to 1 for read operation
 *********************************************************************/
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1;   // Set R/W bit (read mode)
	pI2Cx->DR = SlaveAddr;
}



/*********************************************************************
 * @fn      		  - I2C_ClearADDRFlag
 *
 * @brief             - This function clears ADDR flag by reading SR1 and SR2
 *
 * @param[in]         - Pointer to I2C handle structure
 *
 * @return            - none
 *
 * @Note              - Special handling for single byte reception in master mode
 *********************************************************************/
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummy_read;

	// Check for device mode
	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	{
		// Device is in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize == 1)
			{
				// Disable ACK for single byte reception
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				// Clear ADDR flag (read SR1, read SR2)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}
		}
		else
		{
			// Clear ADDR flag (read SR1, read SR2)
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;
		}
	}
	else
	{
		// Device is in slave mode
		// Clear ADDR flag (read SR1, read SR2)
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;
	}
}



/*********************************************************************
 * @fn      		  - I2C_GenerateStopCondition
 *
 * @brief             - This function generates STOP condition on I2C bus
 *
 * @param[in]         - Base address of I2C peripheral
 *
 * @return            - none
 *
 * @Note              - none
 *********************************************************************/
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}



/*********************************************************************
 * @fn      		  - I2C_SlaveEnableDisableCallbackEvents
 *
 * @brief             - This function enables or disables slave callback events
 *
 * @param[in]         - Base address of I2C peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            - none
 *
 * @Note              - Enables/disables ITEVTEN, ITBUFEN, and ITERREN interrupts
 *********************************************************************/
void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}
	else
	{
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);
	}
}



/*********************************************************************
 * @fn      		  - I2C_PeripheralControl
 *
 * @brief             - This function enables or disables I2C peripheral
 *
 * @param[in]         - Base address of I2C peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            - none
 *
 * @Note              - Controls the PE bit in CR1 register
 *********************************************************************/
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}



/*********************************************************************
 * @fn      		  - I2C_PeriClockControl
 *
 * @brief             - Enables or disables peripheral clock for the given I2C port
 *
 * @param[in]         - Base address of I2C peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            - none
 *
 * @Note              - none
 *********************************************************************/
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
			I2C1_PCLK_EN();
		else if(pI2Cx == I2C2)
			I2C2_PCLK_EN();
		else if(pI2Cx == I2C3)
			I2C3_PCLK_EN();
	}
	else
	{
		// TODO: Implement clock disable macros in stm32f407xx.h
		// I2C1_PCLK_DI(), I2C2_PCLK_DI(), I2C3_PCLK_DI()
	}
}



/*********************************************************************
 * @fn      		  - I2C_Init
 *
 * @brief             - Initializes the given I2C peripheral
 *
 * @param[in]         - Pointer to I2C handle structure
 *
 * @return            - none
 *
 * @Note              - Configures ACK, clock speed, addressing, and timing
 *********************************************************************/
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

	// Enable the clock for I2C peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	// 1. Configure ACK control bit
	tempreg |= (pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK);
	pI2CHandle->pI2Cx->CR1 = tempreg;

	// 2. Configure FREQ field of CR2 register
	tempreg = 0;
	tempreg |= (RCC_GetPCLK1Value() / 1000000U);
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	// 3. Program device own address
	tempreg = 0;
	tempreg |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1);
	tempreg |= (1 << 14);   // Bit 14 should always be kept at 1 by software
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	// 4. CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;

	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// Standard mode
		ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF);
	}
	else
	{
		// Fast mode
		tempreg |= (1 << 15);   // Set F/S bit
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);

		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		else
		{
			ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	// 5. TRISE configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// Standard mode
		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1;
	}
	else
	{
		// Fast mode
		tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}



/*********************************************************************
 * @fn      		  - I2C_DeInit
 *
 * @brief             - Resets all the I2C registers
 *
 * @param[in]         - Base address of I2C peripheral
 *
 * @return            - none
 *
 * @Note              - Uses RCC peripheral reset register
 *********************************************************************/
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	// TODO: Implement I2C reset macros in stm32f407xx.h
	// I2C1_REG_RESET(), I2C2_REG_RESET(), I2C3_REG_RESET()
}



/*********************************************************************
 * @fn      		  - I2C_GetFlagStatus
 *
 * @brief             - This function returns status of given flag
 *
 * @param[in]         - Base address of I2C peripheral
 * @param[in]         - Flag name (macro)
 *
 * @return            - Flag status (SET or RESET)
 *
 * @Note              - Checks SR1 register
 *********************************************************************/
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return SET;
	}
	return RESET;
}



/*********************************************************************
 * @fn      		  - I2C_MasterSendData
 *
 * @brief             - This function sends data from master to slave (blocking)
 *
 * @param[in]         - Pointer to I2C handle structure
 * @param[in]         - Pointer to Tx buffer
 * @param[in]         - Length of data to send
 * @param[in]         - Slave address (7-bit)
 * @param[in]         - Repeated start enable/disable
 *
 * @return            - none
 *
 * @Note              - This is a blocking call (polling mode)
 *********************************************************************/
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	// 1. Generate START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. Confirm START generation by checking SB flag
	//    Note: Until SB is cleared, SCL will be stretched (pulled LOW)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	// 3. Send address of slave with R/W bit set to write (0)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	// 4. Confirm address phase completion by checking ADDR flag
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	// 5. Clear ADDR flag
	//    Note: Until ADDR is cleared, SCL will be stretched (pulled LOW)
	I2C_ClearADDRFlag(pI2CHandle);

	// 6. Send data until Len becomes 0
	while(Len > 0)
	{
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));   // Wait until TXE is set
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	// 7. Wait for TXE=1 and BTF=1 before generating STOP condition
	//    Note: TXE=1, BTF=1 means both SR and DR are empty
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	// 8. Generate STOP condition (if repeated start is disabled)
	if(Sr == I2C_DISABLE_SR)
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}



/*********************************************************************
 * @fn      		  - I2C_MasterReceiveData
 *
 * @brief             - This function receives data from slave (blocking)
 *
 * @param[in]         - Pointer to I2C handle structure
 * @param[in]         - Pointer to Rx buffer
 * @param[in]         - Length of data to receive
 * @param[in]         - Slave address (7-bit)
 * @param[in]         - Repeated start enable/disable
 *
 * @return            - none
 *
 * @Note              - This is a blocking call (polling mode)
 *********************************************************************/
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	// 1. Generate START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. Confirm START generation by checking SB flag
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	// 3. Send address of slave with R/W bit set to read (1)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	// 4. Confirm address phase completion by checking ADDR flag
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	// Procedure to read only 1 byte
	if(Len == 1)
	{
		// Disable ACK
		I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

		// Clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		// Wait until RXNE becomes 1
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

		// Generate STOP condition (if repeated start is disabled)
		if(Sr == I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		// Read data into buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}

	// Procedure to read more than 1 byte
	if(Len > 1)
	{
		// Clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		// Read data until Len becomes 0
		for(uint32_t i = Len; i > 0; i--)
		{
			// Wait until RXNE becomes 1
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

			if(i == 2)   // If last 2 bytes remaining
			{
				// Disable ACK
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				// Generate STOP condition (if repeated start is disabled)
				if(Sr == I2C_DISABLE_SR)
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			// Read data from DR into buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;
			pRxBuffer++;
		}
	}

	// Re-enable ACK
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}
}



/*********************************************************************
 * @fn      		  - I2C_ManageAcking
 *
 * @brief             - This function enables or disables ACK
 *
 * @param[in]         - Base address of I2C peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            - none
 *
 * @Note              - Controls ACK bit in CR1 register
 *********************************************************************/
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}



/*********************************************************************
 * @fn      		  - I2C_IRQInterruptConfig
 *
 * @brief             - This function configures interrupt
 *
 * @param[in]         - IRQ interrupt number
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            - none
 *
 * @Note              - Configures NVIC registers
 *********************************************************************/
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 * @fn      		  - I2C_IRQPriorityConfig
 *
 * @brief             - This function configures interrupt priority
 *
 * @param[in]         - IRQ interrupt number
 * @param[in]         - IRQ interrupt priority
 *
 * @return            - none
 *
 * @Note              - Configures NVIC priority registers
 *********************************************************************/
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}



/*********************************************************************
 * @fn      		  - I2C_MasterSendDataIT
 *
 * @brief             - This function sends data in interrupt mode
 *
 * @param[in]         - Pointer to I2C handle structure
 * @param[in]         - Pointer to Tx buffer
 * @param[in]         - Length of data to send
 * @param[in]         - Slave address (7-bit)
 * @param[in]         - Repeated start enable/disable
 *
 * @return            - State (READY or BUSY)
 *
 * @Note              - This is non-blocking call
 *********************************************************************/
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		// Generate START condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// Enable ITBUFEN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		// Enable ITEVTEN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		// Enable ITERREN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busystate;
}



/*********************************************************************
 * @fn      		  - I2C_MasterReceiveDataIT
 *
 * @brief             - This function receives data in interrupt mode
 *
 * @param[in]         - Pointer to I2C handle structure
 * @param[in]         - Pointer to Rx buffer
 * @param[in]         - Length of data to receive
 * @param[in]         - Slave address (7-bit)
 * @param[in]         - Repeated start enable/disable
 *
 * @return            - State (READY or BUSY)
 *
 * @Note              - This is non-blocking call
 *********************************************************************/
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		// Generate START condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// Enable ITBUFEN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		// Enable ITEVTEN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		// Enable ITERREN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busystate;
}



/*********************************************************************
 * @fn      		  - I2C_MasterHandleTXEInterrupt
 *
 * @brief             - This helper function handles TXE interrupt
 *
 * @param[in]         - Pointer to I2C handle structure
 *
 * @return            - none
 *
 * @Note              - This is a private helper function
 *********************************************************************/
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->TxLen > 0)
	{
		// Load data into DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
		pI2CHandle->TxLen--;
		pI2CHandle->pTxBuffer++;
	}
}



/*********************************************************************
 * @fn      		  - I2C_MasterHandleRXNEInterrupt
 *
 * @brief             - This helper function handles RXNE interrupt
 *
 * @param[in]         - Pointer to I2C handle structure
 *
 * @return            - none
 *
 * @Note              - This is a private helper function
 *********************************************************************/
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->RxSize == 1)
	{
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxLen == 2)
		{
			// Disable ACK
			I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
		}

		// Read data from DR
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxLen == 0)
	{
		// Close I2C data reception and notify application

		// 1. Generate STOP condition (if repeated start is disabled)
		if(pI2CHandle->Sr == I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		// 2. Close I2C Rx
		I2C_CloseReceiveData(pI2CHandle);

		// 3. Notify application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}



/*********************************************************************
 * @fn      		  - I2C_CloseReceiveData
 *
 * @brief             - This function closes I2C reception
 *
 * @param[in]         - Pointer to I2C handle structure
 *
 * @return            - none
 *
 * @Note              - Disables interrupts and resets state
 *********************************************************************/
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	// Disable ITBUFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	// Disable ITEVTEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = 0;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}
}



/*********************************************************************
 * @fn      		  - I2C_CloseSendData
 *
 * @brief             - This function closes I2C transmission
 *
 * @param[in]         - Pointer to I2C handle structure
 *
 * @return            - none
 *
 * @Note              - Disables interrupts and resets state
 *********************************************************************/
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	// Disable ITBUFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	// Disable ITEVTEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = 0;
	pI2CHandle->TxLen = 0;
}



/*********************************************************************
 * @fn      		  - I2C_SlaveSendData
 *
 * @brief             - This function sends data in slave mode
 *
 * @param[in]         - Base address of I2C peripheral
 * @param[in]         - Data to send
 *
 * @return            - none
 *
 * @Note              - Used in slave mode
 *********************************************************************/
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data)
{
	pI2Cx->DR = data;
}



/*********************************************************************
 * @fn      		  - I2C_SlaveReceiveData
 *
 * @brief             - This function receives data in slave mode
 *
 * @param[in]         - Base address of I2C peripheral
 *
 * @return            - Received data
 *
 * @Note              - Used in slave mode
 *********************************************************************/
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
	return (uint8_t)pI2Cx->DR;
}



/*********************************************************************
 * @fn      		  - I2C_EV_IRQHandling
 *
 * @brief             - This function handles I2C event interrupts
 *
 * @param[in]         - Pointer to I2C handle structure
 *
 * @return            - none
 *
 * @Note              - Handles interrupts for both master and slave mode
 *********************************************************************/
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

	// 1. Handle interrupt generated by SB event
	//    Note: SB flag is only applicable in master mode
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);
	if(temp1 && temp3)
	{
		// Interrupt generated because of SB event
		// Execute address phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	// 2. Handle interrupt generated by ADDR event
	//    Note: Master mode - address is sent
	//          Slave mode  - address matched with own address
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	if(temp1 && temp3)
	{
		// Interrupt generated because of ADDR event
		I2C_ClearADDRFlag(pI2CHandle);
	}

	// 3. Handle interrupt generated by BTF (Byte Transfer Finished) event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	if(temp1 && temp3)
	{
		// BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			// Make sure TXE is also set
			if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE))
			{
				// BTF, TXE = 1
				if(pI2CHandle->TxLen == 0)
				{
					// 1. Generate STOP condition (if repeated start is disabled)
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

					// 2. Reset all handle structure members
					I2C_CloseSendData(pI2CHandle);

					// 3. Notify application about transmission complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			// Do nothing
		}
	}

	// 4. Handle interrupt generated by STOPF event
	//    Note: Stop detection flag is applicable only in slave mode
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	if(temp1 && temp3)
	{
		// STOPF flag is set
		// Clear STOPF (read SR1, write to CR1)
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		// Notify application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	// 5. Handle interrupt generated by TXE event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
	if(temp1 && temp2 && temp3)
	{
		// Check for device mode
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			// Master mode
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}
		else
		{
			// Slave mode
			// Make sure slave is in transmitter mode
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}

	// 6. Handle interrupt generated by RXNE event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
	if(temp1 && temp2 && temp3)
	{
		// Check device mode
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			// Master mode
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}
		else
		{
			// Slave mode
			// Make sure slave is in receiver mode
			if(!(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}
}



/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling
 *
 * @brief             - This function handles I2C error interrupts
 *
 * @param[in]         - Pointer to I2C handle structure
 *
 * @return            - none
 *
 * @Note              - Handles bus error, arbitration lost, ACK failure,
 *                      overrun/underrun, and timeout errors
 *********************************************************************/
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1, temp2;

	// Check status of ITERREN control bit in CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & (1 << I2C_CR2_ITERREN);

	// Check for bus error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_BERR);
	if(temp1 && temp2)
	{
		// Clear bus error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);

		// Notify application about error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
	}

	// Check for arbitration lost error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_ARLO);
	if(temp1 && temp2)
	{
		// Clear arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);

		// Notify application about error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
	}

	// Check for ACK failure error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_AF);
	if(temp1 && temp2)
	{
		// Clear ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

		// Notify application about error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}

	// Check for overrun/underrun error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_OVR);
	if(temp1 && temp2)
	{
		// Clear overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);

		// Notify application about error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}

	// Check for timeout error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_TIMEOUT);
	if(temp1 && temp2)
	{
		// Clear timeout error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

		// Notify application about error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}
}



/*********************************************************************
 * @fn      		  - I2C_ApplicationEventCallback
 *
 * @brief             - This is a weak implementation of callback function
 *
 * @param[in]         - Pointer to I2C handle structure
 * @param[in]         - Application event
 *
 * @return            - none
 *
 * @Note              - This function should be overridden by the application
 *********************************************************************/
__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
{
	// This is a weak implementation
	// Application may override this function
}
