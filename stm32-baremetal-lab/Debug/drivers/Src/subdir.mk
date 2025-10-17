################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm32f407xx_gpio.c \
../drivers/Src/stm32f407xx_spi.c 

OBJS += \
./drivers/Src/stm32f407xx_gpio.o \
./drivers/Src/stm32f407xx_spi.o 

C_DEPS += \
./drivers/Src/stm32f407xx_gpio.d \
./drivers/Src/stm32f407xx_spi.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/%.o drivers/Src/%.su drivers/Src/%.cyclo: ../drivers/Src/%.c drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"C:/Users/ÖMER FARUK YESİR/Desktop/STM32F407xx-Drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-drivers-2f-Src

clean-drivers-2f-Src:
	-$(RM) ./drivers/Src/stm32f407xx_gpio.cyclo ./drivers/Src/stm32f407xx_gpio.d ./drivers/Src/stm32f407xx_gpio.o ./drivers/Src/stm32f407xx_gpio.su ./drivers/Src/stm32f407xx_spi.cyclo ./drivers/Src/stm32f407xx_spi.d ./drivers/Src/stm32f407xx_spi.o ./drivers/Src/stm32f407xx_spi.su

.PHONY: clean-drivers-2f-Src

