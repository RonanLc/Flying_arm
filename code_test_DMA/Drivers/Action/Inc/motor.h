#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
//#include <stdio.h>
//#include <stdlib.h>
//#include <math.h>

// Déclaration des variables globales

extern TIM_HandleTypeDef htim4;

// Déclarations des fonctions

void Clock_init(void);
void Motor_init(void);
void MOTOR_Error_Handler(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */

