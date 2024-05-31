
#include "sensor.h"

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

uint8_t IMU_data_buffer[IMU_RECEIVE_DATA_LGTH];
uint8_t IMU_gyro_data[IMU_GYRO_DATA_LGTH][IMU_GYRO_MEAN_VALUE];

double acc[3], gyro[3], angle[3];
double gyro_offset[3] = {0};

// Fonctions
void Sensor_init(void) {

	IMU_init();
}

double Sensor_GetAngle(void) {
	//return
}

double Sensor_GetGyro(void) {
	IMU_Calculate_Gyro();
	return gyro[1];
}

double Sensor_GetMotorSpeed(void) {
	//return
}



void IMU_init(void) {

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

	//HAL_DMA_IRQHandler(&hdma_usart2_rx);

	/* UART2 configuration */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		IMU_Error_Handler();
	}

    // Send initial commands to IMU
    char gyro_reset[3] = {0xFF, 0xAA, 0x52}; // Angle initialization Z-axis to zero

    HAL_UART_Transmit(&huart2, (uint8_t *)gyro_reset, sizeof(gyro_reset), HAL_MAX_DELAY);

    HAL_UART_Receive_DMA(&huart2, IMU_data_buffer, IMU_RECEIVE_DATA_LGTH);

    IMU_init_GyroOffset();
}



uint8_t IMU_Decode_Gyro_Data(void) {

	uint8_t mean_counter = 0;

	for(int i = 0 ; i < IMU_RECEIVE_DATA_LGTH ; i++) {

		if(IMU_data_buffer[i] == 0x55 && IMU_data_buffer[i+1] == 0x52){

			if(mean_counter >= IMU_GYRO_MEAN_VALUE) break;

			for (int j = 0 ; j < IMU_GYRO_DATA_LGTH ; j++) {

				IMU_gyro_data[j][mean_counter] = IMU_data_buffer[i + j];
			}

			mean_counter++;
			i = i + IMU_GYRO_DATA_LGTH - 1;
		}
	}

	return mean_counter;
}


void IMU_Calculate_Gyro(void) {

	uint8_t mean_counter = IMU_Decode_Gyro_Data();
	double mean_gyro = 0;

	for(int i = 0 ; i < mean_counter ; i++) {
		if(IMU_gyro_data[0][i] == 0x55 && IMU_gyro_data[1][i] == 0x52) {

			int16_t temp_w = ((int16_t)IMU_gyro_data[5][0] << 8) | (int16_t)IMU_gyro_data[4][0];
			mean_gyro += (-(((double)temp_w) * 2000.0 / 32768.0) - gyro_offset[1]);
		}
	}
	gyro[1] = mean_gyro / mean_counter;
}


void IMU_Calculate_All_Data(uint8_t IMU_Raw_Data_Buffer[8]) {
    if(IMU_Raw_Data_Buffer[0] == 0x55) {
        int16_t temp_a[3] = {0, 0, 0};
        int16_t temp_w[3] = {0, 0, 0};
        int16_t temp_angle[3] = {0, 0, 0};

        switch(IMU_Raw_Data_Buffer[1]) {
            case 0x51:
                temp_a[0] = ((int16_t)IMU_Raw_Data_Buffer[3] << 8) | (int16_t)IMU_Raw_Data_Buffer[2];
                temp_a[1] = ((int16_t)IMU_Raw_Data_Buffer[5] << 8) | (int16_t)IMU_Raw_Data_Buffer[4];
                temp_a[2] = ((int16_t)IMU_Raw_Data_Buffer[7] << 8) | (int16_t)IMU_Raw_Data_Buffer[6];

                acc[0] = ((double)temp_a[0]) * 16 / 32768.0;
                acc[1] = ((double)temp_a[1]) * 16 / 32768.0;
                acc[2] = ((double)temp_a[2]) * 16 / 32768.0;
                break;
            case 0x52:
                temp_w[0] = ((int16_t)IMU_Raw_Data_Buffer[3] << 8) | (int16_t)IMU_Raw_Data_Buffer[2];
                temp_w[1] = ((int16_t)IMU_Raw_Data_Buffer[5] << 8) | (int16_t)IMU_Raw_Data_Buffer[4];
                temp_w[2] = ((int16_t)IMU_Raw_Data_Buffer[7] << 8) | (int16_t)IMU_Raw_Data_Buffer[6];

                gyro[0] = -(((double)temp_w[0]) * 2000.0 / 32768.0) - gyro_offset[0];
                gyro[1] = -(((double)temp_w[1]) * 2000.0 / 32768.0) - gyro_offset[1];
                gyro[2] = -(((double)temp_w[2]) * 2000.0 / 32768.0) - gyro_offset[2];
                break;
            case 0x53:
                temp_angle[0] = ((int16_t)IMU_Raw_Data_Buffer[3] << 8) | (int16_t)IMU_Raw_Data_Buffer[2];
                temp_angle[1] = ((int16_t)IMU_Raw_Data_Buffer[5] << 8) | (int16_t)IMU_Raw_Data_Buffer[4];
                temp_angle[2] = ((int16_t)IMU_Raw_Data_Buffer[7] << 8) | (int16_t)IMU_Raw_Data_Buffer[6];

                angle[0] = -(((double)temp_angle[0]) * 180.0 / 32768.0) + 90.0;
                angle[1] = -(((double)temp_angle[1]) * 180.0 / 32768.0) + 90.0;
                angle[2] = -(((double)temp_angle[2]) * 180.0 / 32768.0) + 90.0;
                break;
        }
    }
}

void IMU_init_GyroOffset(void) {

	uint8_t flag = 1;

	while(flag == 1) {

		uint8_t mean_counter = 120;
		double gyro_mean[3] = {0.0};

		mean_counter /= IMU_GYRO_MEAN_VALUE;

		for (int i = 0 ; i < mean_counter ; i++) {

			IMU_Decode_Gyro_Data();

			for (int j = 0 ; j < IMU_GYRO_MEAN_VALUE ; j++) {

				uint8_t Temp_gyro_data[IMU_GYRO_DATA_LGTH];

				for (int k = 0 ; k < IMU_GYRO_DATA_LGTH ; k++) {
					Temp_gyro_data[k] = IMU_gyro_data[k][j];
				}

				IMU_Calculate_All_Data(Temp_gyro_data);

				gyro_mean[0] += gyro[0];
				gyro_mean[1] += gyro[1];
				gyro_mean[2] += gyro[2];

			}

			HAL_Delay(5);
		}

		gyro_offset[0] = gyro_mean[0] / (IMU_GYRO_MEAN_VALUE * mean_counter);
		gyro_offset[1] = gyro_mean[1] / (IMU_GYRO_MEAN_VALUE * mean_counter);
		gyro_offset[2] = gyro_mean[2] / (IMU_GYRO_MEAN_VALUE * mean_counter);

		IMU_Decode_Gyro_Data();

		for (int j = 0 ; j < IMU_GYRO_MEAN_VALUE ; j++) {

			uint8_t Temp_gyro_data[IMU_GYRO_DATA_LGTH];

			for (int k = 0 ; k < IMU_GYRO_DATA_LGTH ; k++) {
				Temp_gyro_data[k] = IMU_gyro_data[k][j];
			}

			IMU_Calculate_All_Data(Temp_gyro_data);

			if      (abs(gyro[0]) > 1) { flag = 1; break; }
			else if (abs(gyro[1]) > 1) { flag = 1; break; }
			else if (abs(gyro[2]) > 1) { flag = 1; break; }
			else                       { flag = 0; }

			HAL_Delay(2);
		}
	}
}


void IMU_Error_Handler(void) {

    while(1) {

    }
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(huart->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USART2 DMA Init */
    /* USART2_RX Init */
    hdma_usart2_rx.Instance = DMA1_Stream5;
    hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
    	IMU_Error_Handler();
    }

    __HAL_LINKDMA(huart,hdmarx,hdma_usart2_rx);

  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }

}

void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}
