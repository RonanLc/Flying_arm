
#include "keyboard.h"

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void Keyboard_init(void) {

	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  /* GPIO Ports Clock Enable */
	  __HAL_RCC_GPIOC_CLK_ENABLE();
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(LED_0_GPIO_Port, LED_0_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOA, LED_1_Pin|LED_2_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin : LED_0_Pin */
	  GPIO_InitStruct.Pin = LED_0_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(LED_0_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pins : LED_1_Pin LED_2_Pin */
	  GPIO_InitStruct.Pin = LED_1_Pin|LED_2_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pin : LED_3_Pin */
	  GPIO_InitStruct.Pin = LED_3_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(LED_3_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : BP_green_Pin */
	  GPIO_InitStruct.Pin = BP_green_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	  HAL_GPIO_Init(BP_green_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pins : Keyboard_bit0_Pin Keyboard_bit2_Pin Keyboard_bit1_Pin Keyboard_bit3_Pin */
	  GPIO_InitStruct.Pin = Keyboard_bit0_Pin|Keyboard_bit2_Pin|Keyboard_bit1_Pin|Keyboard_bit3_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pin : Keyboard_IT_Pin */
	  GPIO_InitStruct.Pin = Keyboard_IT_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	  HAL_GPIO_Init(Keyboard_IT_GPIO_Port, &GPIO_InitStruct);

	  /* EXTI interrupt init*/
	  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

char keyboard_translate(void){

	int value = HAL_GPIO_ReadPin(Keyboard_bit0_GPIO_Port, Keyboard_bit0_Pin);
	value += HAL_GPIO_ReadPin(Keyboard_bit1_GPIO_Port, Keyboard_bit1_Pin)*10;
	value += HAL_GPIO_ReadPin(Keyboard_bit2_GPIO_Port, Keyboard_bit2_Pin)*100;
	value += HAL_GPIO_ReadPin(Keyboard_bit3_GPIO_Port, Keyboard_bit3_Pin)*1000;

	switch (value){
		case 0000:
			return '0';
			break;
		case 0001:
			return '1';
			break;
		case 0010:
			return '2';
			break;
		case 0011:
			return '3';
			break;
		case 0100:
			return '4';
			break;
		case 0101:
			return '5';
			break;
		case 0110:
			return '6';
			break;
		case 0111:
			return '7';
			break;
		case 1000:
			return '8';
			break;
		case 1001:
			return '9';
			break;
		case 1010:
			return '*';
			break;
		case 1011:
			return '#';
			break;
		case 1100:
			return 'A';
			break;
		case 1101:
			return 'B';
			break;
		case 1110:
			return 'C';
			break;
		case 1111:
			return 'D';
			break;
		default:
			break;
	}

	return 0;

}

void EXTI2_IRQHandler(void) {
  HAL_GPIO_EXTI_IRQHandler(Keyboard_IT_Pin);
}
