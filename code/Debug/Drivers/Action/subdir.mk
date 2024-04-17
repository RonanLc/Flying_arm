################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Action/lcd.c 

OBJS += \
./Drivers/Action/lcd.o 

C_DEPS += \
./Drivers/Action/lcd.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Action/%.o Drivers/Action/%.su Drivers/Action/%.cyclo: ../Drivers/Action/%.c Drivers/Action/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/Action/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Action

clean-Drivers-2f-Action:
	-$(RM) ./Drivers/Action/lcd.cyclo ./Drivers/Action/lcd.d ./Drivers/Action/lcd.o ./Drivers/Action/lcd.su

.PHONY: clean-Drivers-2f-Action

