
#include "sensor.h"




/*********************************************************/
/*********************** Sensor **************************/
/*********************************************************/

void Sensor_init(void) {

	ADC_init();
	IMU_init();
}

double Sensor_GetAngle(void) {
	return ADC_Calculate_Pot();
}

double Sensor_GetAngleRaw(void) {
	return ADC_data_buffer[99];
}

double Sensor_GetGyro(void) {
	IMU_Calculate_Gyro();
	return gyro[1];
}

double Sensor_GetGyroRaw(void) {

	for(int i = IMU_RECEIVE_DATA_LGTH - IMU_GYRO_DATA_LGTH ; i > 0 ; i--) {

		if(IMU_data_buffer[i] == 0x55 && IMU_data_buffer[i+1] == 0x52){

			int16_t temp_w = ((int16_t)IMU_data_buffer[i + 5] << 8) | (int16_t)IMU_data_buffer[i + 4];
			return (-(((double)temp_w) * 2000.0 / 32768.0) - gyro_offset[1]);
		}
	}

	return -1;
}

double Sensor_GetMotorSpeed(void) {
	return 0;
}

void Sensor_Error_Handler(void) {
    while(1);
}

/*********************************************************/
/************************ Angle **************************/
/*********************************************************/

ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;

uint32_t ADC_data_buffer[ADC_MEAN_VALUE];
uint32_t ADC_Start_Angle;


void ADC_init(void) {

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

	ADC_ChannelConfTypeDef sConfig = {0};

	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc2.Init.Resolution = ADC_RESOLUTION_12B;
	hadc2.Init.ScanConvMode = DISABLE;
	hadc2.Init.ContinuousConvMode = ENABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 1;
	hadc2.Init.DMAContinuousRequests = ENABLE;
	hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

	if (HAL_ADC_Init(&hadc2) != HAL_OK) {
		Sensor_Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	*/
	sConfig.Channel = ADC_CHANNEL_10;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
		Sensor_Error_Handler();
	}

	HAL_ADC_Start_DMA(&hadc2, ADC_data_buffer, ADC_MEAN_VALUE);

	HAL_Delay(1000);

	ADC_init_PotOffset();
}


void ADC_init_PotOffset(void) {

	for(int i = 0; i < ADC_MEAN_VALUE; i++){
		ADC_Start_Angle += ADC_data_buffer[i];
	}

	ADC_Start_Angle /= ADC_MEAN_VALUE;
}

double ADC_Calculate_Pot(void) {

	uint32_t mean_adc_value = 0;

	for(int i = 0; i < ADC_MEAN_VALUE; i++){
		mean_adc_value += ADC_data_buffer[i];
	}

	mean_adc_value /= ADC_MEAN_VALUE;

	double angle;

	if ( mean_adc_value < 1600 ) {
		angle = ADC_COEF_A_L * mean_adc_value + ADC_COEF_B_L;
	}
	else if ( mean_adc_value >= 1600 ) {
		angle = ADC_COEF_A_H * mean_adc_value + ADC_COEF_B_H;
	}

	angle *= M_PI/180;

	return angle;
}


void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc) {

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if(hadc->Instance==ADC2)
  {
    /* Peripheral clock enable */
    __HAL_RCC_ADC2_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**ADC2 GPIO Configuration
    PC0     ------> ADC2_IN10
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* ADC2 DMA Init */
    /* ADC2 Init */
    hdma_adc2.Instance = DMA2_Stream2;
    hdma_adc2.Init.Channel = DMA_CHANNEL_1;
    hdma_adc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc2.Init.Mode = DMA_CIRCULAR;
    hdma_adc2.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc2.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc2) != HAL_OK)
    {
      Sensor_Error_Handler();
    }

    __HAL_LINKDMA(hadc,DMA_Handle,hdma_adc2);
  }
}

void DMA2_Stream2_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_adc2);
}


/*********************************************************/
/************************ Gyro ***************************/
/*********************************************************/

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

uint8_t IMU_data_buffer[IMU_RECEIVE_DATA_LGTH];
uint8_t IMU_gyro_data[IMU_GYRO_DATA_LGTH][IMU_GYRO_MEAN_VALUE];

double acc[3], gyro[3], angle[3];
double gyro_offset[3] = {0};


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
		Sensor_Error_Handler();
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

	gyro[1] = gyro[1] *  M_PI/180;
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


void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if(huart->Instance==USART2) {
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
    	Sensor_Error_Handler();
    }

    __HAL_LINKDMA(huart,hdmarx,hdma_usart2_rx);
  }
}

void DMA1_Stream5_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
}


/*********************************************************/
/************************ Speed **************************/
/*********************************************************/

// TODO: Programmer le capteur de vitesse pour le moteur.
