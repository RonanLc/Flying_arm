#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "keyboard.h"
#include "lcd.h"

//int stateMenu;

void Menu_init(void);
void Menu_refresh(double pot, double gyro);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);




#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
