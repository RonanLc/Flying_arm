
@echo off

set "composant[0]=uart"
set "composant[1]=usart"
set "composant[2]=adc"
set "composant[3]=gpio"

set "nombre=4"






set /a "nombre=%nombre%-1"

for /L %%i in (0 1 %nombre%) do (
    echo %%i
    echo %composant[%%i]%
)