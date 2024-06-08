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

void Motor_Stop(void);
void Motor_Start(void);
void Motor_SetCounterValue(uint32_t counterValue);
void Motor_SetSpeed(uint8_t pourcent);
void Motor_SetTime(uint16_t usTime);

void MOTOR_Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */

