/*
 ******************************************************************************
 * @file           : 002_button_led_toggle.c
 * @author         : Omer Faruk Yesir
 * @brief          : Button-controlled LED toggle using custom GPIO driver (no interrupt)
 ******************************************************************************
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio.h"

/**
 * @brief Simple delay function
 */
void delay(void)
{
	for(uint32_t i = 0; i < 300000; i++); // crude delay loop
}

int main(void)
{
	GPIO_Handle_t GpioLed, GpioButton;

	/* 1. Enable peripheral clocks for GPIOD (LED) and GPIOA (Button) */
	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_PeriClockControl(GPIOA, ENABLE);

	/* 2. Configure LED pin (PD12) */
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GpioLed);

	/* 3. Configure Button pin (PA0) */
	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; // external pull-down on board

	GPIO_Init(&GpioButton);

	uint8_t buttonPressed = 0;

	while(1)
	{
		// Read button state
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0))
		{
			delay(); // debounce delay
			if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0))
			{
				if(buttonPressed == 0)
				{
					GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
					buttonPressed = 1;
				}
			}
		}
		else
		{
			buttonPressed = 0;
		}
	}
}
