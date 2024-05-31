#ifndef __SENSOR_H
#define __SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Déclaration des variables globales

#define IMU_RECEIVE_DATA_LGTH 200
#define IMU_GYRO_DATA_LGTH 8
#define IMU_GYRO_MEAN_VALUE 6

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

extern uint8_t IMU_data_buffer[IMU_RECEIVE_DATA_LGTH];
extern uint8_t IMU_gyro_data[IMU_GYRO_DATA_LGTH][IMU_GYRO_MEAN_VALUE];

// Déclarations de variables pour les mesures
extern double acc[3];
extern double gyro[3];
extern double angle[3];
extern double gyro_offset[3];

// Déclarations des fonctions

void Sensor_init(void);
double Sensor_GetAngle(void);
double Sensor_GetGyro(void);
double Sensor_GetMotorSpeed(void);

void IMU_init(void) ;
uint8_t IMU_Decode_Gyro_Data(void);
void IMU_Calculate_Gyro(void);
void IMU_Calculate_All_Data(uint8_t IMU_Raw_Data_Buffer[8]);
void IMU_init_GyroOffset(void);
void IMU_Error_Handler(void);

void HAL_UART_MspInit(UART_HandleTypeDef* huart);
void DMA1_Stream5_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_H */

