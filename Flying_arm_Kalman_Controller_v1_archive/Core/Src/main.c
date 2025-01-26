/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Fixed-loop sampling time
const double Ts 				= 0.01;   //[s]

// Dynamic model parameters
const float Inertia_Jz 			= 0.0576; //[m/kg]
const float mass_arm_prop 		= 0.240;  //[kg]
const float distOG 				= 0.38;   //[m]
const float gravity 			= 9.81;   //[m/s2]
const float L					= 0.65;   //[m]
const float fr					= 0.1;    //[N.m/s]

// Sensors containers and target
double theta_target 			= M_PI / 2; //[rad]
double theta 					= 0;      //[rad]
double theta_dot				= 0;      //[rad/s]

double PWM_ctrl					= 0;

// Corrector output container
double thrust					= 0; 	  //[N]


/* USER CODE END PV */

/* USER CODE BEGIN PFP */

double map(double val, double min_in, double max_in, double min_out, double max_out);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  //SystemClock_Config();
  Clock_init();
  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  Motor_init();
  Filter_init(Ts);
  Lcd_init();

  // Lcd begining
  Lcd_clear();
  Lcd_cursor(0,0);
  Lcd_string("Starting...");

  HAL_Delay(3000);

  Lcd_clear();

  ADC_init_PotOffset();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  Filter_refresh(thrust);

	  theta = Filter_GetAngle();
	  theta_dot = Filter_GetGyro();

	  thrust = Control_refresh(theta_target, theta, theta_dot);

	  PWM_ctrl = thrust_to_PWM(thrust);

	  Motor_SetTime(PWM_ctrl);

	  // Create a fixed-loop execution //
	  Wait_Until(Ts * 1000);

  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

double map(double val, double min_in, double max_in, double min_out, double max_out){
	 return (val - min_in) * (max_out - min_out) / (max_in - min_in) + min_out;

}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
