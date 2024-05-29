################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Action/Src/imu.c \
../Drivers/Action/Src/lcd.c 

OBJS += \
./Drivers/Action/Src/imu.o \
./Drivers/Action/Src/lcd.o 

C_DEPS += \
./Drivers/Action/Src/imu.d \
./Drivers/Action/Src/lcd.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Action/Src/%.o Drivers/Action/Src/%.su Drivers/Action/Src/%.cyclo: ../Drivers/Action/Src/%.c Drivers/Action/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/Action/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Action-2f-Src

clean-Drivers-2f-Action-2f-Src:
	-$(RM) ./Drivers/Action/Src/imu.cyclo ./Drivers/Action/Src/imu.d ./Drivers/Action/Src/imu.o ./Drivers/Action/Src/imu.su ./Drivers/Action/Src/lcd.cyclo ./Drivers/Action/Src/lcd.d ./Drivers/Action/Src/lcd.o ./Drivers/Action/Src/lcd.su

.PHONY: clean-Drivers-2f-Action-2f-Src

