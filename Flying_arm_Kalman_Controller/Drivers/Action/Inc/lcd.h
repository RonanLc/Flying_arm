/*
 * lcd.h
 *
 *  Created on: 10/06/2018
 *      Author: Olivier Van den Eede
 */

#ifndef LCD_H_
#define LCD_H_

#include "stm32f4xx_hal.h"
#include "string.h"
#include "stdio.h"
#include "main.h"

#define LCD20xN 		// For 20xN LCDs
// #define LCD16xN			// For 16xN LCDs

// For row start addresses
extern const uint8_t ROW_16[];
extern const uint8_t ROW_20[];


/*********************************** LCD Pins connections ***********************************/

#define LCD_data7_Pin GPIO_PIN_7
#define LCD_data7_GPIO_Port GPIOE
#define LCD_data6_Pin GPIO_PIN_8
#define LCD_data6_GPIO_Port GPIOE
#define LCD_data5_Pin GPIO_PIN_9
#define LCD_data5_GPIO_Port GPIOE
#define LCD_data4_Pin GPIO_PIN_15
#define LCD_data4_GPIO_Port GPIOE
#define LCD_EN_Pin GPIO_PIN_11
#define LCD_EN_GPIO_Port GPIOE
#define LCD_RS_Pin GPIO_PIN_13
#define LCD_RS_GPIO_Port GPIOE

/************************************** Command register **************************************/
#define CLEAR_DISPLAY 0x01

#define RETURN_HOME 0x02

#define ENTRY_MODE_SET 0x04
#define OPT_S	0x01					// Shift entire display to right
#define OPT_INC 0x02					// Cursor increment

#define DISPLAY_ON_OFF_CONTROL 0x08
#define OPT_D	0x04					// Turn on display
#define OPT_C	0x02					// Turn on cursor
#define OPT_B 	0x01					// Turn on cursor blink

#define CURSOR_DISPLAY_SHIFT 0x10		// Move and shift cursor
#define OPT_SC 0x08
#define OPT_RL 0x04

#define FUNCTION_SET 0x20
#define OPT_DL 0x10						// Set interface data length
#define OPT_N 0x08						// Set number of display lines
#define OPT_F 0x04						// Set alternate font
#define SETCGRAM_ADDR 0x040
#define SET_DDRAM_ADDR 0x80				// Set DDRAM address


/************************************** LCD defines **************************************/
#define LCD_NIB 4
#define LCD_BYTE 8
#define LCD_DATA_REG 1
#define LCD_COMMAND_REG 0


/************************************** LCD typedefs **************************************/
#define Lcd_PortType GPIO_TypeDef*
#define Lcd_PinType uint16_t

typedef enum {
	LCD_4_BIT_MODE,
	LCD_8_BIT_MODE
} Lcd_ModeTypeDef;


typedef struct {
	Lcd_PortType * data_port;
	Lcd_PinType * data_pin;

	Lcd_PortType rs_port;
	Lcd_PinType rs_pin;

	Lcd_PortType en_port;
	Lcd_PinType en_pin;

	Lcd_ModeTypeDef mode;

} Lcd_HandleTypeDef;

int screen_state;

/************************************** Public functions **************************************/
void Lcd_init();
void Lcd_int(int number);
void Lcd_float(float number);
void Lcd_string(char * string);
void Lcd_cursor(uint8_t row, uint8_t col);
Lcd_HandleTypeDef Lcd_create(
		Lcd_PortType port[], Lcd_PinType pin[],
		Lcd_PortType rs_port, Lcd_PinType rs_pin,
		Lcd_PortType en_port, Lcd_PinType en_pin, Lcd_ModeTypeDef mode);
void Lcd_define_char(uint8_t code, uint8_t bitmap[]);
void Lcd_refresh(keyPress, running);
void Lcd_clear();

#endif /* LCD_H_ */
