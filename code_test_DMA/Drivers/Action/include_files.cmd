@echo off

set "HALversion=1.28.0"

set "HALSrcPath=%USERPROFILE%\STM32Cube\Repository\STM32Cube_FW_F4_V%HALversion%\Drivers\STM32F4xx_HAL_Driver\Src\"
set "HALIncPath=%USERPROFILE%\STM32Cube\Repository\STM32Cube_FW_F4_V%HALversion%\Drivers\STM32F4xx_HAL_Driver\Inc\"

copy "%HALSrcPath%stm32f4xx_hal_usart.c" "..\STM32F4xx_HAL_Driver\Src"
copy "%HALIncPath%stm32f4xx_hal_usart.h" "..\STM32F4xx_HAL_Driver\Inc"

copy "%HALSrcPath%stm32f4xx_hal_uart.c" "..\STM32F4xx_HAL_Driver\Src"
copy "%HALIncPath%stm32f4xx_hal_uart.h" "..\STM32F4xx_HAL_Driver\Inc"

set "filePath=..\..\Core\Inc\stm32f4xx_hal_conf.h"
set "Includefile1=#include "stm32f4xx_hal_uart.c""
set "Includefile2=#include "stm32f4xx_hal_usart.c""

echo %Includefile1% >> %filePath%
echo %Includefile2% >> %filePath%