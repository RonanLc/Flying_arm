#ifndef HMI_H
#define HMI_H

#include "controller.h"
#include "filters.h"
#include "lcd.h"
#include <stdio.h>

// Variables pour les paramètres du système
static MenuState current_menu = MENU_WELCOME;
static ControllerType current_controller = LEADPHASE;
static FilterType current_filter = EKF;
static ConsigneType current_consigne = CONSTANT;

// Types pour les consignes
typedef enum {
	CONSTANT,
	SINUSOIDAL,
	TRIANGULAR,
	RECTANGULAR
} ConsigneType;

// Types de controlleur à utiliser
typedef enum {
	LEADPHASE,
	CASCADE,
	PID
} ControllerType;

// Types de filtres à appliquer
typedef enum {
	AVERAGE,
	EKF,
	NONE
} FilterType;

// États du menu
typedef enum {
    MENU_WELCOME,
    MENU_LAUNCH,
    MENU_INFO,
    MENU_SETUP,
    MENU_SETUP_CONTROLLER,
    MENU_SETUP_FILTER,
    MENU_SETUP_CONSIGNE
} MenuState;

// Initialisation et mise à jour du HMI
void Hmi_init();
void update_hmi();

// Fonctions utilitaires
const char Hmi_get_controller_name(ControllerType ctrl);
const char Hmi_get_filter_name(FilterType filter);
const char Hmi_get_consigne_name(ConsigneType consigne);

// Fonctions internes
static void display_menu();
static void handle_input();

#endif
