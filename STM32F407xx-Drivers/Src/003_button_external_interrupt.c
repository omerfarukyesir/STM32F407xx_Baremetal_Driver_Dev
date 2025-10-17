/*
 ******************************************************************************
 * @file           : 003_button_external_interrupt.c
 * @author         : Omer Faruk Yesir
 * @brief          : External interrupt based button control example
 *                   STM32F407 Discovery (PA0 - User Button, PD12 - LED)
 ******************************************************************************
 */

#include "stm32f407xx_gpio.h"
#include "stm32f407xx.h"
#include <string.h>

void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

int main(void)
{
	GPIO_Handle_t GpioLed, GpioBtn;

	/* ====== LED Configuration (PD12) ====== */
	memset(&GpioLed, 0, sizeof(GpioLed));
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);

	/* ====== Button Configuration (PA0) ====== */
	memset(&GpioBtn, 0, sizeof(GpioBtn));
	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT; // Interrupt at Falling Edge
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioBtn);

	/* ====== IRQ Configuration ====== */
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);

	while(1);
}

/*
 * Interrupt Service Routine for EXTI0 (connected to PA0)
 */
void EXTI0_IRQHandler(void)
{
	delay();  // Debounce delay
	GPIO_IRQHandling(GPIO_PIN_NO_0); // Clear pending bit
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12); // Toggle LED
}
