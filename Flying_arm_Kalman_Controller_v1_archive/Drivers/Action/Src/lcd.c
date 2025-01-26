/*
 * lcd.c
 *
 *  Created on: 10/06/2018
 *      Author: Olivier Van den Eede
 *  Modified on: 29/05/2024
 *      Author: Ronan Le Corronc & Maximilien Kulbicki
 */

#include "lcd.h"

const uint8_t ROW_16[] = {0x00, 0x40, 0x10, 0x50};
const uint8_t ROW_20[] = {0x00, 0x40, 0x14, 0x54};

uint8_t screenSelect = 0;
uint8_t screenActual = -1;
uint8_t cursorSelect = 0;
uint8_t cursorActual = -1;
uint8_t GB_state = 0;

/************************************** Static declarations **************************************/

static void lcd_write_data(uint8_t data);
static void lcd_write_command(uint8_t command);
static void lcd_write(uint8_t data, uint8_t len);

/************************************** LCD pins definitions **************************************/

Lcd_PortType ports[] = {LCD_data4_GPIO_Port, LCD_data5_GPIO_Port, LCD_data6_GPIO_Port, LCD_data7_GPIO_Port};
Lcd_PinType pins[] = {LCD_data4_Pin, LCD_data5_Pin, LCD_data6_Pin, LCD_data7_Pin};

static Lcd_HandleTypeDef lcd;

/************************************** Function definitions **************************************/

/**
 * Create new Lcd_HandleTypeDef and initialize the Lcd
 */
Lcd_HandleTypeDef Lcd_create(
		Lcd_PortType port[], Lcd_PinType pin[],
		Lcd_PortType rs_port, Lcd_PinType rs_pin,
		Lcd_PortType en_port, Lcd_PinType en_pin, Lcd_ModeTypeDef mode)
{
	Lcd_HandleTypeDef lcd;

	lcd.mode = mode;

	lcd.en_pin = en_pin;
	lcd.en_port = en_port;

	lcd.rs_pin = rs_pin;
	lcd.rs_port = rs_port;

	lcd.data_pin = pin;
	lcd.data_port = port;

	return lcd;
}

/**
 * Initialize 16x2-lcd without cursor
 */
void Lcd_init(void) {

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();

	/* Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, LCD_data7_Pin|LCD_data6_Pin|LCD_data5_Pin|LCD_data4_Pin
						  	|LCD_RS_Pin|LCD_EN_Pin, GPIO_PIN_RESET);

	/* Configure GPIO pins for the LCD screen */
	GPIO_InitStruct.Pin = LCD_data7_Pin|LCD_data6_Pin|LCD_data5_Pin|LCD_data4_Pin
						  |LCD_RS_Pin|LCD_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);



	/* LCD configuration */
	lcd = Lcd_create(ports, pins, LCD_RS_GPIO_Port, LCD_RS_Pin, LCD_EN_GPIO_Port, LCD_EN_Pin, LCD_4_BIT_MODE);

	if(lcd.mode == LCD_4_BIT_MODE) {
		lcd_write_command(0x33);
		lcd_write_command(0x32);
		lcd_write_command(FUNCTION_SET | OPT_N);				// 4-bit mode
	}
	else {
		lcd_write_command(FUNCTION_SET | OPT_DL | OPT_N);
	}

	lcd_write_command(CLEAR_DISPLAY);						// Clear screen
	lcd_write_command(DISPLAY_ON_OFF_CONTROL | OPT_D);		// Lcd-on, cursor-off, no-blink
	lcd_write_command(ENTRY_MODE_SET | OPT_INC);			// Increment cursor
}

/**
 * Write a number on the current position
 */
void Lcd_int(int number) {
	char buffer[11];
	sprintf(buffer, "%d", number);

	Lcd_string(buffer);
}

/**
 * Write a floating-point number on the current position
 */
void Lcd_float(float number) {
    char buffer[20];
    snprintf(buffer, sizeof(buffer), "%.6g", number);

    Lcd_string(buffer);
}

/**
 * Write a string on the current position
 */
void Lcd_string(char * string) {
	for(uint8_t i = 0; i < strlen(string); i++) {
			lcd_write_data(string[i]);
	}
}

/**
 * Set the cursor position
 */
void Lcd_cursor(uint8_t row, uint8_t col) {
	#ifdef LCD20xN
	lcd_write_command(SET_DDRAM_ADDR + ROW_20[row] + col);
	#endif

	#ifdef LCD16xN
	lcd_write_command(SET_DDRAM_ADDR + ROW_16[row] + col);
	#endif
}


/**
 * Refresh screen menu
 */
void Lcd_refresh(keyPress, running, consigne) { // GreenButton | keyPress | running | consigne


}


/**
 * Clear the screen
 */
void Lcd_clear(void) {
	lcd_write_command(CLEAR_DISPLAY);
}

void Lcd_define_char(uint8_t code, uint8_t bitmap[]) {
	lcd_write_command(SETCGRAM_ADDR + (code << 3));
	for(uint8_t i=0;i<8;++i){
		lcd_write_data(bitmap[i]);
	}
}


/************************************** Static function definition **************************************/

/**
 * Write a byte to the command register
 */
void lcd_write_command(uint8_t command)
{
	HAL_GPIO_WritePin(lcd.rs_port, lcd.rs_pin, LCD_COMMAND_REG);		// Write to command register

	if(lcd.mode == LCD_4_BIT_MODE)
	{
		lcd_write((command >> 4), LCD_NIB);
		lcd_write(command & 0x0F, LCD_NIB);
	}
	else
	{
		lcd_write(command, LCD_BYTE);
	}

}

/**
 * Write a byte to the data register
 */
void lcd_write_data(uint8_t data)
{
	HAL_GPIO_WritePin(lcd.rs_port, lcd.rs_pin, LCD_DATA_REG);			// Write to data register

	if(lcd.mode == LCD_4_BIT_MODE)
	{
		lcd_write(data >> 4, LCD_NIB);
		lcd_write(data & 0x0F, LCD_NIB);
	}
	else
	{
		lcd_write(data, LCD_BYTE);
	}

}


/**
 * Set len bits on the bus and toggle the enable line
 */
void lcd_write(uint8_t data, uint8_t len)
{
	for(uint8_t i = 0; i < len; i++)
	{
		HAL_GPIO_WritePin(lcd.data_port[i], lcd.data_pin[i], (data >> i) & 0x01);
	}

	HAL_GPIO_WritePin(lcd.en_port, lcd.en_pin, 1);
	HAL_Delay(1);
	HAL_GPIO_WritePin(lcd.en_port, lcd.en_pin, 0); 		// Data receive on falling edge
}
