#ifndef __SENSOR_H
#define __SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>


/*********************************************************/
/*********************** Sensor **************************/
/*********************************************************/

void Sensor_init(void);
double Sensor_GetAngle(void);
double Sensor_GetGyro(void);
double Sensor_GetMotorSpeed(void);
void Sensor_Error_Handler(void);


/*********************************************************/
/************************ Angle **************************/
/*********************************************************/

#define ADC_MEAN_VALUE 100
#define POT_START_ANGLE -40
#define ADC_ZERO_ANGLE 1700

#define ADC_COEF_A_H 0.0891
#define ADC_COEF_B_H -61.5304
#define ADC_COEF_A_L 0.0537
#define ADC_COEF_B_L -4.8767

extern ADC_HandleTypeDef hadc2;
extern DMA_HandleTypeDef hdma_adc2;

extern uint32_t ADC_data_buffer[ADC_MEAN_VALUE];

void ADC_init(void);
double ADC_Calculate_Pot(void);

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc);
void DMA2_Stream2_IRQHandler(void);


/*********************************************************/
/************************ Gyro ***************************/
/*********************************************************/

#define IMU_RECEIVE_DATA_LGTH 200
#define IMU_GYRO_DATA_LGTH 8
#define IMU_GYRO_MEAN_VALUE 6

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

extern uint8_t IMU_data_buffer[IMU_RECEIVE_DATA_LGTH];
extern uint8_t IMU_gyro_data[IMU_GYRO_DATA_LGTH][IMU_GYRO_MEAN_VALUE];

extern double acc[3], gyro[3], angle[3];
extern double gyro_offset[3];


void IMU_init(void);
uint8_t IMU_Decode_Gyro_Data(void);
void IMU_Calculate_Gyro(void);
void IMU_Calculate_All_Data(uint8_t IMU_Raw_Data_Buffer[8]);
void IMU_init_GyroOffset(void);

void HAL_UART_MspInit(UART_HandleTypeDef* huart);
void DMA1_Stream5_IRQHandler(void);


/*********************************************************/
/************************ Speed **************************/
/*********************************************************/

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_H */

