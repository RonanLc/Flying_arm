#include "clavier.h"

char keyboardTranslate(){

	/*HAL_GPIO_WritePin(FA_LED0_GPIO_Port, FA_LED0_Pin, HAL_GPIO_ReadPin(FA_Cbit0_GPIO_Port, FA_Cbit0_Pin));
	HAL_GPIO_WritePin(FA_LED1_GPIO_Port, FA_LED1_Pin, HAL_GPIO_ReadPin(FA_Cbit1_GPIO_Port, FA_Cbit1_Pin));
	HAL_GPIO_WritePin(FA_LED2_GPIO_Port, FA_LED2_Pin, HAL_GPIO_ReadPin(FA_Cbit2_GPIO_Port, FA_Cbit2_Pin));
	HAL_GPIO_WritePin(FA_LED3_GPIO_Port, FA_LED3_Pin, HAL_GPIO_ReadPin(FA_Cbit3_GPIO_Port, FA_Cbit3_Pin));*/

	int value = HAL_GPIO_ReadPin(FA_Cbit0_GPIO_Port, FA_Cbit0_Pin);
	value += HAL_GPIO_ReadPin(FA_Cbit1_GPIO_Port, FA_Cbit1_Pin)*10;
	value += HAL_GPIO_ReadPin(FA_Cbit2_GPIO_Port, FA_Cbit2_Pin)*100;
	value += HAL_GPIO_ReadPin(FA_Cbit3_GPIO_Port, FA_Cbit3_Pin)*1000;

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
