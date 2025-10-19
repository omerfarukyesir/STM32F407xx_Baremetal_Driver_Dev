/*
 ******************************************************************************
 * @file    005_i2c_ds3231_example.c
 * @author  Omer Faruk Yesir
 * @brief   DS3231 RTC test - bare-metal I2C usage
 ******************************************************************************
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio.h"
#include "stm32f407xx_i2c.h"



// DS3231 I2C address and registers
#define DS3231_I2C_ADDRESS        0x68
#define DS3231_REG_SECOND         0x00
#define DS3231_REG_MINUTE         0x01
#define DS3231_REG_HOUR           0x02
#define DS3231_REG_DAY            0x03
#define DS3231_REG_DATE           0x04
#define DS3231_REG_MONTH          0x05
#define DS3231_REG_YEAR           0x06



// Time structure
typedef struct
{
	uint8_t  second;
	uint8_t  minute;
	uint8_t  hour;
	uint8_t  dayOfWeek;
	uint8_t  dayOfMonth;
	uint8_t  month;
	uint16_t year;
} DS3231_Time_t;



// Global variables
I2C_Handle_t I2C1Handle;
DS3231_Time_t currentTime;



// Function prototypes
void I2C1_GPIO_Init(void);
void I2C1_Inits(void);
void delay(void);
uint8_t DS3231_DecToBcd(uint8_t value);
uint8_t DS3231_BcdToDec(uint8_t value);
void DS3231_WriteRegister(uint8_t reg, uint8_t value);
uint8_t DS3231_ReadRegister(uint8_t reg);
void DS3231_SetTime(DS3231_Time_t *pTime);
void DS3231_GetTime(DS3231_Time_t *pTime);
uint8_t I2C_ScanAddress(uint8_t address);



// Simple delay
void delay(void)
{
	for(uint32_t i = 0; i < 500000; i++);
}



// I2C1 GPIO init: PB6=SCL, PB7=SDA
void I2C1_GPIO_Init(void)
{
	GPIO_Handle_t I2CPins;

	// Enable GPIOB clock
	GPIO_PeriClockControl(GPIOB, ENABLE);

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;  // Module has pull-up resistors
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;

	// SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	// SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);
}



// I2C1 init: 100kHz Standard mode
void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = 0x61;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
}



// Decimal to BCD conversion
uint8_t DS3231_DecToBcd(uint8_t value)
{
	return (uint8_t)((value / 10 * 16) + (value % 10));
}



// BCD to Decimal conversion
uint8_t DS3231_BcdToDec(uint8_t value)
{
	return (uint8_t)((value / 16 * 10) + (value % 16));
}



// Write single register to DS3231
void DS3231_WriteRegister(uint8_t reg, uint8_t value)
{
	uint8_t data[2];
	data[0] = reg;
	data[1] = value;

	I2C_MasterSendData(&I2C1Handle, data, 2, DS3231_I2C_ADDRESS, I2C_DISABLE_SR);
}



// Read single register from DS3231
uint8_t DS3231_ReadRegister(uint8_t reg)
{
	uint8_t data;

	// Send register address with repeated start
	I2C_MasterSendData(&I2C1Handle, &reg, 1, DS3231_I2C_ADDRESS, I2C_ENABLE_SR);

	// Read data
	I2C_MasterReceiveData(&I2C1Handle, &data, 1, DS3231_I2C_ADDRESS, I2C_DISABLE_SR);

	return data;
}



// Set time on DS3231
void DS3231_SetTime(DS3231_Time_t *pTime)
{
	uint8_t data[8];

	data[0] = DS3231_REG_SECOND;
	data[1] = DS3231_DecToBcd(pTime->second);
	data[2] = DS3231_DecToBcd(pTime->minute);
	data[3] = DS3231_DecToBcd(pTime->hour);
	data[4] = DS3231_DecToBcd(pTime->dayOfWeek);
	data[5] = DS3231_DecToBcd(pTime->dayOfMonth);
	data[6] = DS3231_DecToBcd(pTime->month);
	data[7] = DS3231_DecToBcd(pTime->year - 2000);

	I2C_MasterSendData(&I2C1Handle, data, 8, DS3231_I2C_ADDRESS, I2C_DISABLE_SR);
}



// Get time from DS3231
void DS3231_GetTime(DS3231_Time_t *pTime)
{
	uint8_t data[7];
	uint8_t reg = DS3231_REG_SECOND;

	// Send starting register address with repeated start
	I2C_MasterSendData(&I2C1Handle, &reg, 1, DS3231_I2C_ADDRESS, I2C_ENABLE_SR);

	// Read 7 bytes
	I2C_MasterReceiveData(&I2C1Handle, data, 7, DS3231_I2C_ADDRESS, I2C_DISABLE_SR);

	// Convert BCD to decimal
	pTime->second = DS3231_BcdToDec(data[0] & 0x7F);
	pTime->minute = DS3231_BcdToDec(data[1] & 0x7F);
	pTime->hour = DS3231_BcdToDec(data[2] & 0x3F);
	pTime->dayOfWeek = DS3231_BcdToDec(data[3] & 0x07);
	pTime->dayOfMonth = DS3231_BcdToDec(data[4] & 0x3F);
	pTime->month = DS3231_BcdToDec(data[5] & 0x1F);
	pTime->year = DS3231_BcdToDec(data[6]) + 2000;
}



// I2C address scanner - test if device responds
uint8_t I2C_ScanAddress(uint8_t address)
{
	uint8_t dummy = 0;

	// Try to send to the address
	I2C_MasterSendData(&I2C1Handle, &dummy, 1, address, I2C_DISABLE_SR);

	// If no error, device is present
	return 1;
}



int main(void)
{
	// I2C GPIO init
	I2C1_GPIO_Init();

	// I2C peripheral init
	I2C1_Inits();

	// Enable I2C
	I2C_PeripheralControl(I2C1, ENABLE);

	// Enable ACK
	I2C_ManageAcking(I2C1, ENABLE);


	// Test: Check if DS3231 is connected (set breakpoint here and check result)
	uint8_t deviceFound = I2C_ScanAddress(DS3231_I2C_ADDRESS);
	(void)deviceFound;  // Suppress unused warning
	// deviceFound should be 1 if DS3231 is connected


	// Set time (use only on first run, then comment out)
	/*
	DS3231_Time_t setTime;
	setTime.second = 0;
	setTime.minute = 30;
	setTime.hour = 14;
	setTime.dayOfWeek = 1;
	setTime.dayOfMonth = 20;
	setTime.month = 10;
	setTime.year = 2025;
	DS3231_SetTime(&setTime);
	*/


	// Main loop - continuously read time
	while(1)
	{
		DS3231_GetTime(&currentTime);

		// Debug: set breakpoint and watch currentTime variable
		// currentTime.hour, currentTime.minute, currentTime.second etc.

		// Wait 3 seconds
		for(int i = 0; i < 3; i++)
		{
			delay();
		}
	}

	return 0;
}
