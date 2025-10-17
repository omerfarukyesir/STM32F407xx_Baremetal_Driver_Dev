/*
 ******************************************************************************
 * @file    stm32f407xx_spi.h
 * @author  Omer Faruk Yesir
 * @brief   Header file for SPI driver (Low Layer) for STM32F407xx
 *
 ******************************************************************************
 */

#ifndef INC_STM32F407XX_SPI_H_
#define INC_STM32F407XX_SPI_H_

#include "stm32f407xx.h"

/*
 * Configuration structure for SPIx peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode;       /*!< Master or Slave @SPI_DEVICE_MODES */
	uint8_t SPI_BusConfig;        /*!< Full-Duplex, Half-Duplex, Simplex @SPI_BUS_CONFIG */
	uint8_t SPI_SclkSpeed;        /*!< Clock speed prescaler @SPI_SCLK_SPEED */
	uint8_t SPI_DFF;              /*!< Data Frame Format @SPI_DFF (8/16 bit) */
	uint8_t SPI_CPOL;             /*!< Clock Polarity @SPI_CPOL */
	uint8_t SPI_CPHA;             /*!< Clock Phase @SPI_CPHA */
	uint8_t SPI_SSM;              /*!< Software slave management @SPI_SSM (NSS) */
} SPI_Config_t;


/*
 * Handle structure for SPIx peripheral
 */
typedef struct
{
	SPI_RegDef_t *pSPIx;          /*!< Base address of SPI peripheral */
	SPI_Config_t SPIConfig;       /*!< SPI configuration settings */
	uint8_t *pTxBuffer;           /*!< Pointer to Tx buffer (for IT) */
	uint8_t *pRxBuffer;           /*!< Pointer to Rx buffer (for IT) */
	uint32_t TxLen;               /*!< Tx length (for IT) */
	uint32_t RxLen;               /*!< Rx length (for IT) */
	uint8_t TxState;              /*!< Transmission state (READY / BUSY) */
	uint8_t RxState;              /*!< Reception state (READY / BUSY) */
} SPI_Handle_t;


/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_SLAVE     0
#define SPI_DEVICE_MODE_MASTER    1


/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD         1     /*!< Full duplex */
#define SPI_BUS_CONFIG_HD         2     /*!< Half duplex (1-line) */
#define SPI_BUS_CONFIG_SIMPLEX_RX 3     /*!< Simplex RX only */


/*
 * @SPI_SclkSpeed (BR bits in CR1)
 * Values correspond to BR[2:0] = 000 .. 111
 */
#define SPI_SCLK_SPEED_DIV2       0
#define SPI_SCLK_SPEED_DIV4       1
#define SPI_SCLK_SPEED_DIV8       2
#define SPI_SCLK_SPEED_DIV16      3
#define SPI_SCLK_SPEED_DIV32      4
#define SPI_SCLK_SPEED_DIV64      5
#define SPI_SCLK_SPEED_DIV128     6
#define SPI_SCLK_SPEED_DIV256     7


/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS             0
#define SPI_DFF_16BITS            1


/*
 * @SPI_CPOL
 */
#define SPI_CPOL_LOW              0
#define SPI_CPOL_HIGH             1


/*
 * @SPI_CPHA
 */
#define SPI_CPHA_LOW              0
#define SPI_CPHA_HIGH             1


/*
 * @SPI_SSM
 */
#define SPI_SSM_DI                0
#define SPI_SSM_EN                1


/*
 * SPI states
 */
#define SPI_READY                 0
#define SPI_BUSY_IN_RX            1
#define SPI_BUSY_IN_TX            2


/*
 * Possible SPI application events
 */
#define SPI_EVENT_TX_CMPLT        1
#define SPI_EVENT_RX_CMPLT        2
#define SPI_EVENT_OVR_ERR         3
#define SPI_EVENT_CRC_ERR         4


/*********************************************************************
 * 						API FUNCTION PROTOTYPES
 *********************************************************************/
/* Peripheral Clock setup */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/* Init and De-Init */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/* Data Send and Receive (blocking / polling) */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/* Data Send and Receive (interrupt / non-blocking) */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/* IRQ Configuration and ISR handling */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/* Other Peripheral Control APIs */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/* Utility: flag status */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

/* Application Callback (weak - user may override) */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);


#endif /* INC_STM32F407XX_SPI_H_ */
