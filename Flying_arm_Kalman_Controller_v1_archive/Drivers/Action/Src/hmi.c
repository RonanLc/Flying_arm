

void Screen_print() {



	if(GreenButton == 1 && GB_state){
		running = (running+1)%2;
		GB_state = 0;
	}
	if(GreenButton == 0 && !GB_state){
		GB_state = 1;
	}

	Key_setup();
	Screen_plot();
	Cursor_plot();

}

// newValue (flag si on édit une valeur ou pas)
void Key_setup() {

	switch(keyPress){
		case 'A':
			switch(screen_state){
				case 0 :
					screen_state = 1;
				case 1 :
					if(cursorSelect == 1)
					screen_state = 2;
					else if(cursorSelect == 2)
					screen_state = 5;
				case 3 :
					if(newValue) {
						newValue = 0;
					}
					else {
						newValue = 1;
					}
			}

		case 'B':
			if (screenSelect == 2 || screenSelect == 3 || screenSelect == 4 || screenSelect == 5) {
				screenSelect = 1;
				running = 0;
			}

		case '2':
			if (cursorSelect == 2) {
				cursorSelect = 1;
			}

		case '8':
			if (cursorSelect == 1) {
				cursorSelect = 2;
			}

		case '4':
			if () {

			}

		case '6':
			if () {

			}
	}

}


void Cursor_plot() {

	if(cursorActual != cursorSelect && screenSelect > 0){
		cursorActual = cursorSelect;
		switch(cursorSelect){
			case 0:
				Lcd_cursor(0,0);
				Lcd_string(" ");
				Lcd_cursor(0,1);
				Lcd_string(" ");
			case 1:
				Lcd_cursor(0,0);
				Lcd_string(">");
				Lcd_cursor(0,1);
				Lcd_string(" ");
			case 2:
				Lcd_cursor(0,0);
				Lcd_string(" ");
				Lcd_cursor(0,1);
				Lcd_string(">");
		}
	}
}


void Screen_plot() {

	if(screenActual != screenSelect || running){
		screenActual = screenSelect;

		switch(screenSelect){
			case 0:
				Lcd_clean();
				Lcd_cursor(0,0);
				Lcd_string("Welcome - Flying Arm");
				Lcd_cursor(0,0);
				Lcd_string("Press A to continue");

			case 1:
				cursorSelect = 1;
				Lcd_clean();
				Lcd_cursor(1,0);
				Lcd_string("Launch");
				Lcd_cursor(1,0);
				Lcd_string("Setup");

			case 3:
				break;

			case 4:
				cursorSelect = 0;
				Lcd_cursor(0,0);
				Lcd_string("Setpoint: ");
				Lcd_int(consigne);
				if(running) {
					Lcd_cursor(0,16);
					Lcd_string(" RUN");
				}
				else {
					Lcd_cursor(0,15);
					Lcd_string(" STOP");
				}
				Lcd_cursor(0,0);
				Lcd_string("Press GB start/stop");
		}
	}
}





