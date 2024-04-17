################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Action/Src/FA_utilities.c \
../Drivers/Action/Src/clavier.c \
../Drivers/Action/Src/lcd.c \
../Drivers/Action/Src/mpu6050.c 

OBJS += \
./Drivers/Action/Src/FA_utilities.o \
./Drivers/Action/Src/clavier.o \
./Drivers/Action/Src/lcd.o \
./Drivers/Action/Src/mpu6050.o 

C_DEPS += \
./Drivers/Action/Src/FA_utilities.d \
./Drivers/Action/Src/clavier.d \
./Drivers/Action/Src/lcd.d \
./Drivers/Action/Src/mpu6050.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Action/Src/%.o Drivers/Action/Src/%.su Drivers/Action/Src/%.cyclo: ../Drivers/Action/Src/%.c Drivers/Action/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/Action/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Action-2f-Src

clean-Drivers-2f-Action-2f-Src:
	-$(RM) ./Drivers/Action/Src/FA_utilities.cyclo ./Drivers/Action/Src/FA_utilities.d ./Drivers/Action/Src/FA_utilities.o ./Drivers/Action/Src/FA_utilities.su ./Drivers/Action/Src/clavier.cyclo ./Drivers/Action/Src/clavier.d ./Drivers/Action/Src/clavier.o ./Drivers/Action/Src/clavier.su ./Drivers/Action/Src/lcd.cyclo ./Drivers/Action/Src/lcd.d ./Drivers/Action/Src/lcd.o ./Drivers/Action/Src/lcd.su ./Drivers/Action/Src/mpu6050.cyclo ./Drivers/Action/Src/mpu6050.d ./Drivers/Action/Src/mpu6050.o ./Drivers/Action/Src/mpu6050.su

.PHONY: clean-Drivers-2f-Action-2f-Src

