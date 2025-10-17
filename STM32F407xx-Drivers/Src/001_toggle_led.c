/*
 ******************************************************************************
 * @file           : 001_toggle_led.c
 * @author         : Omer Faruk Yesir
 * @brief          : Simple LED Blinking using Custom GPIO Driver (Low Layer)
 ******************************************************************************
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio.h"

/**
 * @brief Simple delay function
 */
void delay(void)
{
	for(uint32_t i = 0; i < 500000; i++); // crude delay loop
}

int main(void)
{
	GPIO_Handle_t GpioLed;

	/* 1. Enable the peripheral clock for GPIOD */
	GpioLed.pGPIOx = GPIOD;
	GPIO_PeriClockControl(GpioLed.pGPIOx, ENABLE);

	/* 2. Configure LED pins */
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;        // Output mode
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;        // Fast speed
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;       // Push-pull output
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;     // No pull-up, no pull-down

	/* 3. Initialize LED pins: PD12, PD13, PD14, PD15 */
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&GpioLed);

	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&GpioLed);

	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&GpioLed);

	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&GpioLed);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12); // Green LED
		delay();

		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_13); // Orange LED
		delay();

		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_14); // Red LED
		delay();

		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_15); // Blue LED
		delay();
	}
}
