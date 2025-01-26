
#include "math.h"
#include "hmi.h"

#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif


/*********************************************************/
/*************** Parametrage du controller ***************/
/*********************************************************/

// Dynamic model parameters
extern const float Inertia_Jz; 		//[m/kg]
extern const float mass_arm_prop;  	//[kg]
extern const float distOG;   		//[m]
extern const float gravity;   		//[m/s2]
extern const float L;   			//[m]
extern const float fr;    			//[N.m/s]

extern const double Ts ;
char control_selection = 2;

void Control_init();
void Control_selection();
double Control_refresh(ControllerType Controller_type, double theta_target, double theta_measure, double theta_dot_measure){


// PWM function parameters
const float T_pwm_offset 		= 1100;
const float alpha_pwm 			= 1.0567E-5;

double min_max(double val, double min, double max);
double thrust_to_PWM(double thrust);

double theta_error 				= 0;      //[rad]
double new_thrust				= 0;	  //[N]
const float max_thrust			= 2.76;   //[N]

/*********************************************************/
/************** Controleur a avance de phase *************/
/*********************************************************/

// Lead-phase controller parameters
const float a_ldc 				= 5.828;  //[1]
const float t_ldc 				= 0.07935;//[s]
double c1 						= 0;      //[1]
double c2 						= 0;      //[1]
double c3 						= 0;      //[1]
double c4 						= 0;      //[1]
double output_ldc[2] 			= {0,0};
double input_ldc[2] 			= {0,0};

void leadphase_init();
double leadphase_control(double theta_target, double theta_measure);


/*********************************************************/
/***************** Controleur en cascade *****************/
/*********************************************************/


// Sensor-feed controller parameters
const float K_theta 			= 4;      //[1]
const float K_dot_theta			= 1;      //[1]

void cascade_init();
double cascade_control(double theta_target, double theta_measure, double theta_dot_measure);


/*********************************************************/
/******************** Controleur PID *********************/
/*********************************************************/




#ifdef __cplusplus
}
#endif

#endif /* __CONTROLLER_H */
