#ifndef __IMU_H
#define __IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <math.h>

// Déclaration des variables globales
extern uint8_t IMU_Raw_Data_Buffer[11];
extern unsigned char IMU_Data_Buffer_counter;
extern UART_HandleTypeDef huart2;

// Déclarations de variables pour les mesures
extern double acc[3];
extern double gyro[3];
extern double angle[3];
extern double gyro_offset[3];
extern volatile int _GyroValueUpdated;
extern volatile int _IsGyroIntialized;

// Déclarations des fonctions
void Sensor_init();
void Decode_IMU_Data(uint8_t IMU_Raw_Data_Buffer[11]);
void GyroOffset_Init(void);
void Imu_Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_H */

