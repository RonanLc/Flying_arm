
#include "imu.h"

// Déclarations de variables globales
uint8_t IMU_Raw_Data_Buffer[11];
unsigned char IMU_Data_Buffer_counter;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

double acc[3] = {0};
double gyro[3] = {0};
double angle[3] = {0};
double gyro_offset[3] = {0};
volatile int _GyroValueUpdated = 0;
volatile int _IsGyroIntialized = 0;

// Fonctions
void Sensor_init() {

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
		Imu_Error_Handler();
	}

    // Send initial commands to IMU
    char gyro_reset[3] = {0xFF, 0xAA, 0x52}; // Angle initialization Z-axis to zero

    HAL_UART_Transmit(&huart2, (uint8_t *)gyro_reset, sizeof(gyro_reset), HAL_MAX_DELAY);

}

/*void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);

	if(huart->Instance == USART2) {
        IMU_Raw_Data_Buffer[IMU_Data_Buffer_counter] = (char)(huart->Instance->DR & (uint8_t)0x00FF);

        if(IMU_Data_Buffer_counter == 0 && IMU_Raw_Data_Buffer[0] != 0x55) return;

        IMU_Data_Buffer_counter++;

        if(IMU_Data_Buffer_counter == 11) {
            IMU_Data_Buffer_counter = 0;
            Decode_IMU_Data(IMU_Raw_Data_Buffer);
        }

        // Enable reception again
        HAL_UART_Receive_IT(&huart2, (uint8_t*)&IMU_Raw_Data_Buffer[IMU_Data_Buffer_counter], 1);
    }
}*/

void Decode_IMU_Data(uint8_t IMU_Raw_Data_Buffer[11]) {
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

                _GyroValueUpdated = 1;
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

void GyroOffset_Init(void) {
    while(_GyroValueUpdated != 1);
    if(_GyroValueUpdated == 1) {
        uint8_t counter = 0;
        double gyro_mean[3] = {0.0};

        _IsGyroIntialized = 0;
        do {
            if(_GyroValueUpdated == 1) {
                gyro_mean[0] += gyro[0];
                gyro_mean[1] += gyro[1];
                gyro_mean[2] += gyro[2];
                counter++;
                _GyroValueUpdated = 0;
            }
        } while(counter < 50);

        gyro_offset[0] = gyro_mean[0] / 50.0;
        gyro_offset[1] = gyro_mean[1] / 50.0;
        gyro_offset[2] = gyro_mean[2] / 50.0;

        counter = 0;

        do {
            if(_GyroValueUpdated == 1) {
                _GyroValueUpdated = 0;
                if(gyro[0] > 1.0) return;
                if(gyro[1] > 1.0) return;
                if(gyro[2] > 1.0) return;
                counter++;
            }
        } while(counter < 50);

        _IsGyroIntialized = 1;
    }
}

void Imu_Error_Handler(void) {

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);

    while(1) {

    }
}
