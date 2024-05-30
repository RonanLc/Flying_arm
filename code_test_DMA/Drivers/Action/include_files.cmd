@echo off

set "HALversion=1.28.0"

set "composant[0]=uart"
set "composant[1]=usart"
set "composant[2]=adc"
set "composant[3]=gpio"

set "nombreComposant=4"











set "HALSrcPath=%USERPROFILE%\STM32Cube\Repository\STM32Cube_FW_F4_V%HALversion%\Drivers\STM32F4xx_HAL_Driver\Src\"
set "HALIncPath=%USERPROFILE%\STM32Cube\Repository\STM32Cube_FW_F4_V%HALversion%\Drivers\STM32F4xx_HAL_Driver\Inc\"

copy "%HALSrcPath%stm32f4xx_hal_usart.c" "..\STM32F4xx_HAL_Driver\Src"
copy "%HALIncPath%stm32f4xx_hal_usart.h" "..\STM32F4xx_HAL_Driver\Inc"

copy "%HALSrcPath%stm32f4xx_hal_uart.c" "..\STM32F4xx_HAL_Driver\Src"
copy "%HALIncPath%stm32f4xx_hal_uart.h" "..\STM32F4xx_HAL_Driver\Inc"


rem Définit les fichiers à traiter
set "filePath=..\..\Core\Inc\"

set "confFile=stm32f4xx_hal_conf.h"
set "tempFile=temp_hal_conf.h"

set "confPath=%filePath%%confFile%"
set "tempPath=%filePath%%tempFile%"

rem Définit les lignes à détecter et à remplacer
set "ligne_a_remplacer=/* #define HAL_UART_MODULE_ENABLED */"
set "nouvelle_ligne=#define HAL_UART_MODULE_ENABLED"

rem Efface le fichier de sortie s'il existe déjà
if exist "%tempPath%" del "%tempPath%"

rem Désactiver l'expansion retardée
setlocal disabledelayedexpansion

rem Parcourir chaque ligne du fichier d'entrée
for /f "tokens=1* delims=:" %%a in ('findstr /n "^" "%confPath%"') do (
    
    set "ligne=%%b"

    rem Activer l'expansion retardée pour la comparaison et l'écriture
    setlocal EnableDelayedExpansion
    rem Comparer la ligne courante avec la ligne à remplacer
    if "!ligne!"=="%ligne_a_remplacer%" (
        rem Si la ligne correspond, écrire la nouvelle ligne
        >> "%tempPath%" echo %nouvelle_ligne%
    ) else (
        rem Sinon, écrire la ligne courante inchangée
        >> "%tempPath%" echo(!ligne!
    )
    endlocal
)

endlocal

del "%confPath%"
rename "%tempPath%" "%confFile%"

rem Efface le fichier de sortie si jamais il existe toujours
if exist "%tempPath%" del "%tempPath%"