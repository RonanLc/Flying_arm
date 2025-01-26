
#ifndef CLAVIER_H_
#define CLAVIER_H_

#include "stm32f4xx_hal.h"

#define BP_green_Pin GPIO_PIN_3
#define BP_green_GPIO_Port GPIOC

#define Keyboard_bit0_Pin GPIO_PIN_1
#define Keyboard_bit0_GPIO_Port GPIOA
#define Keyboard_bit1_Pin GPIO_PIN_4
#define Keyboard_bit1_GPIO_Port GPIOA
#define Keyboard_bit2_Pin GPIO_PIN_3
#define Keyboard_bit2_GPIO_Port GPIOA
#define Keyboard_bit3_Pin GPIO_PIN_6
#define Keyboard_bit3_GPIO_Port GPIOA
#define Keyboard_IT_Pin GPIO_PIN_2
#define Keyboard_IT_GPIO_Port GPIOA
#define Keyboard_IT_EXTI_IRQn EXTI2_IRQn

#define LED_0_Pin GPIO_PIN_1
#define LED_0_GPIO_Port GPIOC
#define LED_1_Pin GPIO_PIN_0
#define LED_1_GPIO_Port GPIOA
#define LED_2_Pin GPIO_PIN_5
#define LED_2_GPIO_Port GPIOA
#define LED_3_Pin GPIO_PIN_11
#define LED_3_GPIO_Port GPIOB

void Keyboard_init(void);
char keyboard_translate(void);

extern void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void EXTI2_IRQHandler(void);

#endif /* CLAVIER_H_ */
