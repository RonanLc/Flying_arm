#ifndef SETPOINT_H
#define SETPOINT_H

#include "math.h"
#include "hmi.h"

// Variables du système
double delta_T;
double T;

// Variables génération de signaux
double Period_Sinusoidal = 10;
double Period_Triangular = 10;
double Period_Rectangular = 10;

double Lower_setpoint = M_PI / 2;
double Upper_setpoint = 3 * M_PI / 4;

// Initialisation et mise à jour du HMI
void Setpoint_init();
double Setpoint_GetTarget(ConsigneType Consigne_type, double last_theta_target);


#endif
