/*
 *******************************************************************************
 * @file    stm32f407xx_spi.c
 * @author  Omer Faruk Yesir
 * @brief   SPI driver implementation for STM32F407xx (Low Level)
 *******************************************************************************
 */

#include "stm32f407xx_spi.h"
#include <stddef.h>

/* Static helper prototypes */
static void SPI_Txe_Interrupt_Handle(SPI_Handle_t *pSPIHandle);
static void SPI_Rxne_Interrupt_Handle(SPI_Handle_t *pSPIHandle);
static void SPI_Ovr_Interrupt_Handle(SPI_Handle_t *pSPIHandle);

/*****************************************************************
 * @fn          - SPI_PeriClockControl
 *
 * @brief       - This function enables or disables peripheral
 *                clock for the given SPI port
 *
 * @param[in]   - pSPIx   : Base address of the SPI peripheral
 * @param[in]   - EnorDi  : ENABLE or DISABLE macro
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        if(pSPIx == SPI1)
        {
            SPI1_PCLK_EN();
        }
        else if(pSPIx == SPI2)
        {
            SPI2_PCLK_EN();
        }
        else if(pSPIx == SPI3)
        {
            SPI3_PCLK_EN();
        }
    }
    else
    {
        if(pSPIx == SPI1)
        {
            SPI1_PCLK_DI();
        }
        else if(pSPIx == SPI2)
        {
            SPI2_PCLK_DI();
        }
        else if(pSPIx == SPI3)
        {
            SPI3_PCLK_DI();
        }
    }
}

/*****************************************************************
 * @fn          - SPI_Init
 *
 * @brief       - This function initializes the SPI peripheral
 *                according to the settings in SPI_Handle_t
 *
 * @param[in]   - pSPIHandle : Pointer to SPI handle structure
 *
 * @return      - None
 *
 * @Note        - Configures device mode, bus config, SCLK speed,
 *                DFF, CPOL, CPHA, and SSM bits.
 *
 *****************************************************************/
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
    uint32_t tempreg = 0;

    /* Enable SPI peripheral clock */
    SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

    /* 1. Configure device mode */
    tempreg |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);

    /* 2. Configure bus config */
    if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
    {
        tempreg &= ~(1U << SPI_CR1_BIDIMODE);
    }
    else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
    {
        tempreg |= (1U << SPI_CR1_BIDIMODE);
    }
    else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX)
    {
        tempreg &= ~(1U << SPI_CR1_BIDIMODE);
        tempreg |= (1U << SPI_CR1_RXONLY);
    }

    /* 3. Configure SCLK speed */
    tempreg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

    /* 4. Configure DFF */
    tempreg |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

    /* 5. Configure CPOL and CPHA */
    tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);
    tempreg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

    /* 6. Configure SSM */
    tempreg |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);

    /* Write to CR1 register */
    pSPIHandle->pSPIx->CR1 = tempreg;
}

/*****************************************************************
 * @fn          - SPI_DeInit
 *
 * @brief       - This function resets the SPI peripheral registers
 *
 * @param[in]   - pSPIx : Base address of the SPI peripheral
 *
 * @return      - None
 *
 * @Note        - Uses the respective peripheral reset macros.
 *
 *****************************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
    if(pSPIx == SPI1)
    {
        SPI1_REG_RESET();
    }
    else if(pSPIx == SPI2)
    {
        SPI2_REG_RESET();
    }
    else if(pSPIx == SPI3)
    {
        SPI3_REG_RESET();
    }
}

/*****************************************************************
 * @fn          - SPI_GetFlagStatus
 *
 * @brief       - This function checks if a given flag in SPI
 *                status register is set
 *
 * @param[in]   - pSPIx     : Base address of the SPI peripheral
 * @param[in]   - FlagName  : Name of the flag to check
 *
 * @return      - SET or RESET
 *
 * @Note        - None
 *
 *****************************************************************/
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
    if(pSPIx->SR & FlagName)
    {
        return SET;
    }
    return RESET;
}

/*****************************************************************
 * @fn          - SPI_SendData
 *
 * @brief       - Sends data over SPI in blocking mode
 *
 * @param[in]   - pSPIx     : Base address of the SPI peripheral
 * @param[in]   - pTxBuffer : Pointer to transmit buffer
 * @param[in]   - Len       : Length of data to send
 *
 * @return      - None
 *
 * @Note        - Waits for TXE flag before sending each byte/word.
 *                Waits until BSY flag is cleared at the end.
 *
 *****************************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
    while(Len > 0U)
    {
        while(SPI_GetFlagStatus(pSPIx, (1U << SPI_SR_TXE)) == RESET);

        if(pSPIx->CR1 & (1U << SPI_CR1_DFF))
        {
            pSPIx->DR = *((uint16_t*)pTxBuffer);
            pTxBuffer += 2;
            Len -= 2;
        }
        else
        {
            pSPIx->DR = *pTxBuffer;
            pTxBuffer++;
            Len--;
        }
    }

    while(SPI_GetFlagStatus(pSPIx, (1U << SPI_SR_BSY)) == SET);
}

/*****************************************************************
 * @fn          - SPI_ReceiveData
 *
 * @brief       - Receives data over SPI in blocking mode
 *
 * @param[in]   - pSPIx     : Base address of the SPI peripheral
 * @param[in]   - pRxBuffer : Pointer to receive buffer
 * @param[in]   - Len       : Length of data to receive
 *
 * @return      - None
 *
 * @Note        - Waits for RXNE flag before reading each byte/word.
 *
 *****************************************************************/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
    while(Len > 0U)
    {
        while(SPI_GetFlagStatus(pSPIx, (1U << SPI_SR_RXNE)) == RESET);

        if(pSPIx->CR1 & (1U << SPI_CR1_DFF))
        {
            *((uint16_t*)pRxBuffer) = (uint16_t)pSPIx->DR;
            pRxBuffer += 2;
            Len -= 2;
        }
        else
        {
            *pRxBuffer = (uint8_t)pSPIx->DR;
            pRxBuffer++;
            Len--;
        }
    }
}

/*****************************************************************
 * @fn          - SPI_SendDataIT
 *
 * @brief       - Sends data over SPI using interrupts
 *
 * @param[in]   - pSPIHandle : Pointer to SPI handle structure
 * @param[in]   - pTxBuffer  : Pointer to transmit buffer
 * @param[in]   - Len        : Length of data to send
 *
 * @return      - 1 if transmission started, 0 if SPI is busy
 *
 * @Note        - Enables TXEIE interrupt in CR2.
 *
 *****************************************************************/
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
    if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
    {
        pSPIHandle->pTxBuffer = pTxBuffer;
        pSPIHandle->TxLen = Len;
        pSPIHandle->TxState = SPI_BUSY_IN_TX;

        pSPIHandle->pSPIx->CR2 |= (1U << SPI_CR2_TXEIE);

        return 1;
    }
    return 0;
}

/*****************************************************************
 * @fn          - SPI_ReceiveDataIT
 *
 * @brief       - Receives data over SPI using interrupts
 *
 * @param[in]   - pSPIHandle : Pointer to SPI handle structure
 * @param[in]   - pRxBuffer  : Pointer to receive buffer
 * @param[in]   - Len        : Length of data to receive
 *
 * @return      - 1 if reception started, 0 if SPI is busy
 *
 * @Note        - Enables RXNEIE interrupt in CR2.
 *
 *****************************************************************/
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
    if(pSPIHandle->RxState != SPI_BUSY_IN_RX)
    {
        pSPIHandle->pRxBuffer = pRxBuffer;
        pSPIHandle->RxLen = Len;
        pSPIHandle->RxState = SPI_BUSY_IN_RX;

        pSPIHandle->pSPIx->CR2 |= (1U << SPI_CR2_RXNEIE);

        return 1;
    }
    return 0;
}

/*****************************************************************
 * @fn          - SPI_PeripheralControl
 *
 * @brief       - Enables or disables SPI peripheral
 *
 * @param[in]   - pSPIx   : Base address of the SPI peripheral
 * @param[in]   - EnorDi  : ENABLE or DISABLE macro
 *
 * @return      - None
 *
 * @Note        - Sets or clears SPE bit in CR1.
 *
 *****************************************************************/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        pSPIx->CR1 |= (1U << SPI_CR1_SPE);
    }
    else
    {
        pSPIx->CR1 &= ~(1U << SPI_CR1_SPE);
    }
}

/*****************************************************************
 * @fn          - SPI_SSIConfig
 *
 * @brief       - Configures the SSI bit
 *
 * @param[in]   - pSPIx   : Base address of the SPI peripheral
 * @param[in]   - EnorDi  : ENABLE or DISABLE macro
 *
 * @return      - None
 *
 * @Note        - Needed when SSM is enabled to avoid MODF error.
 *
 *****************************************************************/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        pSPIx->CR1 |= (1U << SPI_CR1_SSI);
    }
    else
    {
        pSPIx->CR1 &= ~(1U << SPI_CR1_SSI);
    }
}

/*****************************************************************
 * @fn          - SPI_SSOEConfig
 *
 * @brief       - Configures the SSOE bit
 *
 * @param[in]   - pSPIx   : Base address of the SPI peripheral
 * @param[in]   - EnorDi  : ENABLE or DISABLE macro
 *
 * @return      - None
 *
 * @Note        - Automatically manages NSS pin in master mode.
 *
 *****************************************************************/
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        pSPIx->CR2 |= (1U << SPI_CR2_SSOE);
    }
    else
    {
        pSPIx->CR2 &= ~(1U << SPI_CR2_SSOE);
    }
}

/*****************************************************************
 * @fn          - SPI_IRQInterruptConfig
 *
 * @brief       - Enables or disables the NVIC interrupt for SPI
 *
 * @param[in]   - IRQNumber : IRQ number
 * @param[in]   - EnorDi    : ENABLE or DISABLE macro
 *
 * @return      - None
 *
 * @Note        - Uses NVIC_ISER/ICER registers.
 *
 *****************************************************************/
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        if(IRQNumber <= 31)
        {
            *NVIC_ISER0 |= (1U << IRQNumber);
        }
        else if(IRQNumber > 31 && IRQNumber < 64)
        {
            *NVIC_ISER1 |= (1U << (IRQNumber % 32));
        }
        else if(IRQNumber >= 64 && IRQNumber < 96)
        {
            *NVIC_ISER2 |= (1U << (IRQNumber % 64));
        }
    }
    else
    {
        if(IRQNumber <= 31)
        {
            *NVIC_ICER0 |= (1U << IRQNumber);
        }
        else if(IRQNumber > 31 && IRQNumber < 64)
        {
            *NVIC_ICER1 |= (1U << (IRQNumber % 32));
        }
        else if(IRQNumber >= 64 && IRQNumber < 96)
        {
            *NVIC_ICER2 |= (1U << (IRQNumber % 64));
        }
    }
}

/*****************************************************************
 * @fn          - SPI_IRQPriorityConfig
 *
 * @brief       - Configures the priority of SPI interrupt in NVIC
 *
 * @param[in]   - IRQNumber  : IRQ number
 * @param[in]   - IRQPriority: Priority value
 *
 * @return      - None
 *
 * @Note        - Adjusts shift amount based on implemented bits.
 *
 *****************************************************************/
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;
    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
    *(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/*****************************************************************
 * @fn          - SPI_IRQHandling
 *
 * @brief       - Handles SPI interrupts for TXE, RXNE, OVR
 *
 * @param[in]   - pSPIHandle : Pointer to SPI handle structure
 *
 * @return      - None
 *
 * @Note        - Calls respective internal helper functions.
 *
 *****************************************************************/
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
    uint8_t temp1, temp2;

    /* TXE */
    temp1 = (pSPIHandle->pSPIx->SR & (1U << SPI_SR_TXE)) ? 1 : 0;
    temp2 = (pSPIHandle->pSPIx->CR2 & (1U << SPI_CR2_TXEIE)) ? 1 : 0;
    if(temp1 && temp2)
    {
        SPI_Txe_Interrupt_Handle(pSPIHandle);
    }

    /* RXNE */
    temp1 = (pSPIHandle->pSPIx->SR & (1U << SPI_SR_RXNE)) ? 1 : 0;
    temp2 = (pSPIHandle->pSPIx->CR2 & (1U << SPI_CR2_RXNEIE)) ? 1 : 0;
    if(temp1 && temp2)
    {
        SPI_Rxne_Interrupt_Handle(pSPIHandle);
    }

    /* OVR */
    temp1 = (pSPIHandle->pSPIx->SR & (1U << SPI_SR_OVR)) ? 1 : 0;
    temp2 = (pSPIHandle->pSPIx->CR2 & (1U << SPI_CR2_ERRIE)) ? 1 : 0;
    if(temp1 && temp2)
    {
        SPI_Ovr_Interrupt_Handle(pSPIHandle);
    }
}

/*****************************************************************
 * @fn          - SPI_Txe_Interrupt_Handle
 *
 * @brief       - Internal helper to handle TXE interrupt
 *
 * @param[in]   - pSPIHandle : Pointer to SPI handle structure
 *
 * @return      - None
 *
 * @Note        - Sends next byte/word and calls callback on completion.
 *
 *****************************************************************/
static void SPI_Txe_Interrupt_Handle(SPI_Handle_t *pSPIHandle)
{
    if(pSPIHandle->pSPIx->CR1 & (1U << SPI_CR1_DFF))
    {
        pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
        pSPIHandle->pTxBuffer += 2;
        if(pSPIHandle->TxLen >= 2)
        {
            pSPIHandle->TxLen -= 2;
        }
        else
        {
            pSPIHandle->TxLen = 0;
        }
    }
    else
    {
        pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
        pSPIHandle->pTxBuffer++;
        if(pSPIHandle->TxLen > 0)
        {
            pSPIHandle->TxLen--;
        }
    }

    if(pSPIHandle->TxLen == 0)
    {
        pSPIHandle->pSPIx->CR2 &= ~(1U << SPI_CR2_TXEIE);
        pSPIHandle->TxState = SPI_READY;
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
    }
}

/*****************************************************************
 * @fn          - SPI_Rxne_Interrupt_Handle
 *
 * @brief       - Internal helper to handle RXNE interrupt
 *
 * @param[in]   - pSPIHandle : Pointer to SPI handle structure
 *
 * @return      - None
 *
 * @Note        - Reads next byte/word and calls callback on completion.
 *
 *****************************************************************/
static void SPI_Rxne_Interrupt_Handle(SPI_Handle_t *pSPIHandle)
{
    if(pSPIHandle->pSPIx->CR1 & (1U << SPI_CR1_DFF))
    {
        *((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
        pSPIHandle->pRxBuffer += 2;
        if(pSPIHandle->RxLen >= 2)
        {
            pSPIHandle->RxLen -= 2;
        }
        else
        {
            pSPIHandle->RxLen = 0;
        }
    }
    else
    {
        *(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
        pSPIHandle->pRxBuffer++;
        if(pSPIHandle->RxLen > 0)
        {
            pSPIHandle->RxLen--;
        }
    }

    if(pSPIHandle->RxLen == 0)
    {
        pSPIHandle->pSPIx->CR2 &= ~(1U << SPI_CR2_RXNEIE);
        pSPIHandle->RxState = SPI_READY;
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
    }
}

/*****************************************************************
 * @fn          - SPI_Ovr_Interrupt_Handle
 *
 * @brief       - Internal helper to handle overrun (OVR) interrupt
 *
 * @param[in]   - pSPIHandle : Pointer to SPI handle structure
 *
 * @return      - None
 *
 * @Note        - Reads DR and SR to clear OVR flag.
 *
 *****************************************************************/
static void SPI_Ovr_Interrupt_Handle(SPI_Handle_t *pSPIHandle)
{
    uint8_t temp;
    if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
    {
        temp = (uint8_t)pSPIHandle->pSPIx->DR;
        temp = (uint8_t)pSPIHandle->pSPIx->SR;
        (void)temp;
    }
    SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

/*****************************************************************
 * @fn          - SPI_ApplicationEventCallback
 *
 * @brief       - Weak function to be implemented by user
 *
 * @param[in]   - pSPIHandle : Pointer to SPI handle structure
 * @param[in]   - AppEv      : SPI event
 *
 * @return      - None
 *
 * @Note        - Called on TX/RX complete or OVR error
 *
 *****************************************************************/
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
    /* User can override this function in application */
}
