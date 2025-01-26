
#include "menu.h"

int stateMenu = 0;

void Menu_init(void) {

	  Lcd_init();
	  Keyboard_init();

	  Lcd_clear();
	  Lcd_cursor(0,0);
	  Lcd_string("Starting...");

	  HAL_Delay(500);

	  stateMenu = 0;

	  Lcd_clear();

}

void Menu_refresh(double gyro, double pot) {

	Lcd_cursor(0, 0);
	Lcd_float(pot);
	Lcd_string(" | ");
	Lcd_float(gyro);
	Lcd_string("           ");

	Lcd_cursor(1, 0);
	Lcd_int(stateMenu);
	Lcd_string("    ");

}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	char key = keyboard_translate();

	if(key == 'A'){
		stateMenu += 10;
		if(stateMenu > 40){stateMenu -= 10;}
	}
	if(key == 'B'){
		stateMenu -= 10;
		if(stateMenu < 0){stateMenu += 10;}
	}
	if(key == 'C'){
		stateMenu += 1;
		if(stateMenu%10 > 5){stateMenu -= 1;}
	}
	if(key == 'D'){
		stateMenu -= 1;
		if(stateMenu%10 < 0){stateMenu += 1;}
	}

}
