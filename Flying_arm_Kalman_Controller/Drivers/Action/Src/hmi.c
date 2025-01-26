
#include "hmi.h"

// Initialisation du HMI
void Hmi_init() {
	Lcd_init();
    current_menu = MENU_WELCOME;
    display_menu();
}

// Mise à jour du HMI
void update_hmi() {
    handle_input();
    display_menu();
}

// Affichage du menu
static void display_menu() {
    char buffer[40]; // Buffer pour l'affichage sur 2x20 caractères

    switch (current_menu) {
        case MENU_WELCOME:
            snprintf(buffer, sizeof(buffer), "Welcome to System");
            Lcd_clear();
            Lcd_string(buffer);
            break;

        case MENU_LAUNCH:
            snprintf(buffer, sizeof(buffer), "1. Info\n2. Back");
            Lcd_clear();
            Lcd_string(buffer);
            break;

        case MENU_INFO:
            snprintf(buffer, sizeof(buffer), "Setpoint:%s\nStart/Stop:%d",
                     get_consigne_name(current_consigne),
                     is_system_running());
            Lcd_clear();
            Lcd_string(buffer);
            break;

        case MENU_SETUP:
            snprintf(buffer, sizeof(buffer), "1.Controller\n2.Filter\n3.Setpoint");
            Lcd_clear();
            Lcd_string(buffer);
            break;

        case MENU_SETUP_CONTROLLER:
            snprintf(buffer, sizeof(buffer), "Controller:%s\n1. Back",
                     get_controller_name(current_controller));
            Lcd_clear();
            Lcd_string(buffer);
            break;

        case MENU_SETUP_FILTER:
            snprintf(buffer, sizeof(buffer), "Filter:%s\n1. Back",
                     get_filter_name(current_filter));
            lcd_clear();
            lcd_print(buffer);
            break;

        case MENU_SETUP_CONSIGNE:
            snprintf(buffer, sizeof(buffer), "Setpoint:%s\n1. Back",
                     get_consigne_name(current_consigne));
            Lcd_clear();
            Lcd_string(buffer);
            break;
    }
}

// Gestion des entrées utilisateur
static void handle_input() {
    int key = keypad_read(); // Lecture des touches
    switch (current_menu) {
        case MENU_WELCOME:
            if (key == 'A') current_menu = MENU_LAUNCH;
            else if (key == 'B') current_menu = MENU_SETUP;
            break;

        case MENU_LAUNCH:
            if (key == 'A') current_menu = MENU_INFO;
            else if (key == 'B') current_menu = MENU_WELCOME;
            break;

        case MENU_INFO:
            if (key == 'A') toggle_system_start_stop();
            else if (key == 'B') current_menu = MENU_LAUNCH;
            break;

        case MENU_SETUP:
            if (key == 'A') current_menu = MENU_SETUP_CONTROLLER;
            else if (key == 'B') current_menu = MENU_SETUP_FILTER;
            else if (key == '3') current_menu = MENU_SETUP_CONSIGNE;
            break;

        case MENU_SETUP_CONTROLLER:
            if (key == 'A') current_controller = (current_controller == LEADPHASE) ? CASCADE : LEADPHASE;
            else if (key == 'B') current_menu = MENU_SETUP;
            break;

        case MENU_SETUP_FILTER:
            if (key == 'A') current_filter = (current_filter == EKF) ? AVERAGE : EKF;
            else if (key == 'B') current_menu = MENU_SETUP;
            break;

        case MENU_SETUP_CONSIGNE:
            if (key == 'A') current_consigne = (current_consigne == CONSTANT) ? SINUSOIDAL : CONSTANT;
            else if (key == 'B') current_menu = MENU_SETUP;
            break;
    }
}


// ### UTILITIES ### //

const char Hmi_get_controller_name(ControllerType ctrl) {
    // return (ctrl == LEADPHASE) ? "Phase Advance" : "Cascade";
    return current_controller;
}

const char Hmi_get_filter_name(FilterType filter) {
    // return (filter == EKF) ? "EKF" : "Moving Average";
    return current_filter;
}

const char Hmi_get_consigne_name(ConsigneType consigne) {
    /*switch (consigne) {
        case CONSTANT: return "Constant";
        case SINUSOIDAL: return "Sinusoidal";
        case TRIANGULAR: return "Triangular";
        case RECTANGULAR: return "Rectangular";
        default: return "Unknown";
    }*/
    return current_consigne;
}

// ### CALLBACK ### //

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if(GPIO_Pin == Keyboard_IT_Pin) {
	  update_hmi();
  }
}


/*

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
*/





