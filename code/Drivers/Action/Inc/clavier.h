
#include "stm32f4xx_hal.h"

#define FA_Cbit0_Pin GPIO_PIN_4
#define FA_Cbit0_GPIO_Port GPIOA
#define FA_Cbit1_Pin GPIO_PIN_3
#define FA_Cbit1_GPIO_Port GPIOA
#define FA_Cbit2_Pin GPIO_PIN_6
#define FA_Cbit2_GPIO_Port GPIOA
#define FA_Cbit3_Pin GPIO_PIN_1
#define FA_Cbit3_GPIO_Port GPIOA

#define FA_LED0_Pin GPIO_PIN_1
#define FA_LED0_GPIO_Port GPIOC
#define FA_LED1_Pin GPIO_PIN_0
#define FA_LED1_GPIO_Port GPIOA
#define FA_LED2_Pin GPIO_PIN_5
#define FA_LED2_GPIO_Port GPIOA
#define FA_LED3_Pin GPIO_PIN_11
#define FA_LED3_GPIO_Port GPIOB

#ifndef CLAVIER_H_
#define CLAVIER_H_

char keyboardTranslate();

#endif /* CLAVIER_H_ */
