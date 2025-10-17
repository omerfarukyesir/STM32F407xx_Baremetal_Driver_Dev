/*
 ******************************************************************************
 * @file    stm32f407xx_gpio.h
 * @author  Omer Faruk Yesir
 * @brief   Header file for GPIO driver (Low Layer)
 *
 ******************************************************************************
 */
#ifndef INC_STM32F407XX_GPIO_H_
#define INC_STM32F407XX_GPIO_H_

#include "stm32f407xx.h"



/*  Configuration Structure of GPIO*/
typedef struct
{
	uint8_t GPIO_PinNumber;           /*!< Possible values from @GPIO_PIN_NUMBERS 			*/
	uint8_t GPIO_PinMode;             /*!< Possible values from @GPIO_PIN_MODES 			*/
	uint8_t GPIO_PinSpeed;            /*!< Possible values from @GPIO_PIN_SPEED 			*/
	uint8_t GPIO_PinPuPdControl;      /*!< Possible values from @GPIO_PIN_PUPD 			*/
	uint8_t GPIO_PinOPType;           /*!< Possible values from @GPIO_PIN_OUTPUT_TYPES 	*/
	uint8_t GPIO_PinAltFunMode;       /*!< Alternate function mode (if applicable) 		*/
}GPIO_PinConfig_t;



/* Handle structure for a GPIO pin*/
typedef struct
{
	GPIO_RegDef_t *pGPIOx;            /*!< Base address of the GPIO port to which pin belongs */
	GPIO_PinConfig_t GPIO_PinConfig;  /*!< GPIO pin configuration settings */
}GPIO_Handle_t;



/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0                0
#define GPIO_PIN_NO_1                1
#define GPIO_PIN_NO_2                2
#define GPIO_PIN_NO_3                3
#define GPIO_PIN_NO_4                4
#define GPIO_PIN_NO_5                5
#define GPIO_PIN_NO_6                6
#define GPIO_PIN_NO_7                7
#define GPIO_PIN_NO_8                8
#define GPIO_PIN_NO_9                9
#define GPIO_PIN_NO_10               10
#define GPIO_PIN_NO_11               11
#define GPIO_PIN_NO_12               12
#define GPIO_PIN_NO_13               13
#define GPIO_PIN_NO_14               14
#define GPIO_PIN_NO_15               15



/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_INPUT              0
#define GPIO_MODE_OUTPUT             1
#define GPIO_MODE_ALTFN              2
#define GPIO_MODE_ANALOG             3
#define GPIO_MODE_IT_FT              4   /*!< Falling edge trigger interrupt */
#define GPIO_MODE_IT_RT              5   /*!< Rising edge trigger interrupt */
#define GPIO_MODE_IT_RFT             6   /*!< Rising and Falling edge trigger interrupt */



/*
 * @GPIO_PIN_OUTPUT_TYPES
 */
#define GPIO_OP_TYPE_PP              0   /*!< Push-Pull */
#define GPIO_OP_TYPE_OD              1   /*!< Open-Drain */



/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW               0
#define GPIO_SPEED_MEDIUM            1
#define GPIO_SPEED_FAST              2
#define GPIO_SPEED_HIGH              3



/*
 * @GPIO_PIN_PULL_UP_PULL_DOWN
 * GPIO pin pull-up and pull-down configuration macros
 */
#define GPIO_NO_PUPD                 0
#define GPIO_PIN_PU                  1
#define GPIO_PIN_PD                  2




/*
 * APIs supported by this driver
 * For more information about the APIs, check the function definitions in stm32f407xx_gpio.c file
 */
/* Peripheral Clock setup */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);



/* Init and De-init */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);



/* Data read and write */
uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);



/* IRQ Configuration and ISR handling */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);



#endif /* INC_STM32F407XX_GPIO_H_ */
