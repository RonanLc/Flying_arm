/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "FA_utilities.h"
#include "clavier.h"
#include "lcd.h"
//#include "mpu6050.h"
#include "imu.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FA_PotArm_Pin GPIO_PIN_0
#define FA_PotArm_GPIO_Port GPIOC
#define FA_LED0_Pin GPIO_PIN_1
#define FA_LED0_GPIO_Port GPIOC
#define FA_PotBox_Pin GPIO_PIN_2
#define FA_PotBox_GPIO_Port GPIOC
#define FA_BP_Pin GPIO_PIN_3
#define FA_BP_GPIO_Port GPIOC
#define FA_LED1_Pin GPIO_PIN_0
#define FA_LED1_GPIO_Port GPIOA
#define FA_Cbit3_Pin GPIO_PIN_1
#define FA_Cbit3_GPIO_Port GPIOA
#define FA_CbitInterrupt_Pin GPIO_PIN_2
#define FA_CbitInterrupt_GPIO_Port GPIOA
#define FA_CbitInterrupt_EXTI_IRQn EXTI2_IRQn
#define FA_Cbit1_Pin GPIO_PIN_3
#define FA_Cbit1_GPIO_Port GPIOA
#define FA_Cbit0_Pin GPIO_PIN_4
#define FA_Cbit0_GPIO_Port GPIOA
#define FA_LED2_Pin GPIO_PIN_5
#define FA_LED2_GPIO_Port GPIOA
#define FA_Cbit2_Pin GPIO_PIN_6
#define FA_Cbit2_GPIO_Port GPIOA
#define FA_Data7_Pin GPIO_PIN_7
#define FA_Data7_GPIO_Port GPIOE
#define FA_Data6_Pin GPIO_PIN_8
#define FA_Data6_GPIO_Port GPIOE
#define FA_Data5_Pin GPIO_PIN_9
#define FA_Data5_GPIO_Port GPIOE
#define FA_EN_Pin GPIO_PIN_11
#define FA_EN_GPIO_Port GPIOE
#define FA_RS_Pin GPIO_PIN_13
#define FA_RS_GPIO_Port GPIOE
#define FA_Data4_Pin GPIO_PIN_15
#define FA_Data4_GPIO_Port GPIOE
#define FA_LED3_Pin GPIO_PIN_11
#define FA_LED3_GPIO_Port GPIOB
#define FA_ESC_PWM_Pin GPIO_PIN_12
#define FA_ESC_PWM_GPIO_Port GPIOD
#define FA_SDA_IMU_Pin GPIO_PIN_9
#define FA_SDA_IMU_GPIO_Port GPIOC
#define FA_SCL_IMU_Pin GPIO_PIN_8
#define FA_SCL_IMU_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
