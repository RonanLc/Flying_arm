#include "controller.h"


/*********************************************************/
/**************** Selection du controller ****************/
/*********************************************************/

void Control_init(){



}


void Control_selection(){



}


double Control_refresh(ControllerType Controller_type, double theta_target, double theta_measure, double theta_dot_measure){

	control_selection = Controller_type;

	switch (control_selection) {
	    case PID:
	    	break;
	    case CASCADE:
	    	return cascade_control(theta_target, theta_measure, theta_dot_measure);
	    case LEADPHASE:
	    	return leadphase_control(theta_target, theta_measure);
	}

	return -1;
}



double min_max(double val, double min, double max){
	if (val > max){
		return max;
	}
	if (val < min){
		return min;
	}
	else{
		return val;
	}

}

/*
 * Returns the appropriate PWM length to attain the given thrust
 * */
double thrust_to_PWM(double thrust){
	return T_pwm_offset + sqrt(thrust / alpha_pwm);
}

/*********************************************************/
/************** Controleur a avance de phase *************/
/*********************************************************/


void leadphase_init();

double leadphase_control(double theta_target, double theta_measure){

	c1 = Ts + 2*t_ldc;
	c2 = Ts - 2*t_ldc;
	c3 = Ts + 2*a_ldc*t_ldc;
	c4 = Ts - 2*a_ldc*t_ldc;

	// Error evaluation //
	theta_error = theta_target - theta_measure;
	//theta_error = theta_target - theta_est;

	// Plant linearisation //
	new_thrust = distOG*mass_arm_prop*gravity*sin(theta_measure)/L;

	// Update input and compute output //
	input_ldc[1] = theta_error;
	output_ldc[1] = (input_ldc[1] * c3 + input_ldc[0] * c4 - output_ldc[0] * c2)/c1;

	// Correction of the thrust //
	new_thrust += output_ldc[1];

	// Shift the in/out //
	input_ldc[0] = input_ldc[1];
	output_ldc[0] = output_ldc[1];

	// Bound the thrust to its caracteristics //
	new_thrust = min_max(new_thrust, 0, max_thrust);

	// Set the corresponding PWM //
	return new_thrust;

}



/*********************************************************/
/***************** Controleur en cascade *****************/
/*********************************************************/

void cascade_init(){



}

double cascade_control(double theta_target, double theta_measure, double theta_dot_measure){

	// Error evaluation //
	theta_error = theta_target - theta_measure;
	//theta_error = theta_target - theta_est;

	// Plant linearisation //
	new_thrust = distOG*mass_arm_prop*gravity*sin(theta_measure)/L;

	// Sensor-feed controller //
	new_thrust += K_dot_theta *( K_theta * theta_error - theta_dot_measure);

	// Bound the thrust to its caracteristics //
	new_thrust = min_max(new_thrust, 0, max_thrust);

	// Set the corresponding PWM //
	return new_thrust;

}


/*********************************************************/
/******************** Controleur PID *********************/
/*********************************************************/

// En developpement, le PID est non efficace pour ce système
