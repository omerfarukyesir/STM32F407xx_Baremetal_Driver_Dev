/*
 ******************************************************************************
 * @file    006_usart2_tx_test.c
 * @author  Omer Faruk Yesir
 * @brief   USART2 Transmit test with button trigger
 *
 ******************************************************************************
 */

#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"
#include "stm32f407xx_gpio.h"
#include "stm32f407xx_usart.h"


// Test message
char msg[1024] = "UART Tx testing...\n\r";

// USART handle
USART_Handle_t usart2_handle;


// Simple delay
void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}


// USART2 GPIO init: PA2=TX, PA3=RX
void USART2_GPIOInit(void)
{
	GPIO_Handle_t usart_gpios;

	// Enable GPIOA clock
	GPIO_PeriClockControl(GPIOA, ENABLE);

	usart_gpios.pGPIOx = GPIOA;
	usart_gpios.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usart_gpios.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usart_gpios.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	usart_gpios.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	usart_gpios.GPIO_PinConfig.GPIO_PinAltFunMode = 7;

	// USART2 TX - PA2
	usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIO_Init(&usart_gpios);

	// USART2 RX - PA3
	usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&usart_gpios);
}


// USART2 init: 115200 baud, 8N1
void USART2_Init(void)
{
	usart2_handle.pUSARTx = USART2;
	usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart2_handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;

	USART_Init(&usart2_handle);
}


// Button and LED GPIO init
void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn, GpioLed;

	// Button GPIO - PA0
	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);

	// LED GPIO - PD12
	GPIO_PeriClockControl(GPIOD, ENABLE);

	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GpioLed);
}


int main(void)
{
	// Initialize button and LED
	GPIO_ButtonInit();

	// Initialize USART2 GPIO
	USART2_GPIOInit();

	// Initialize USART2 peripheral
	USART2_Init();

	// Enable USART2
	USART_PeripheralControl(USART2, ENABLE);

	while(1)
	{
		// Wait for button press (PA0)
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		// Debounce delay
		delay();

		// Toggle LED
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);

		// Send message via USART2
		USART_SendData(&usart2_handle, (uint8_t*)msg, strlen(msg));
	}

	return 0;
}
