
#include "setpoint.h"

// Setup variation speed

void Setpoint_init(double Ts){
	delta_T = Ts;
}

//

double Setpoint_GetTarget(ConsigneType Consigne_type, double last_theta_target){


	switch (Consigne_type) {
		case CONSTANT:
			return last_theta_target;

		case SINUSOIDAL:
			T += delta_T;

			if (T > Period_Sinusoidal) { T = 0; }

			return ((Upper_setpoint - Lower_setpoint) / 2) * (1 + sin(2*M_PI * Period_Sinusoidal * T));

		case TRIANGULAR:
			T += delta_T;

			if (T > Period_Triangular) { T = 0; }

			if (T < Period_Triangular/2) {
				return (Upper_setpoint / (Period_Triangular/2)) * T + Lower_setpoint; }
			else {
				return -(Upper_setpoint / (Period_Triangular/2)) * T + (Upper_setpoint*2);
			}

		case RECTANGULAR:
			T += delta_T;

			if (T > Period_Rectangular) { T = 0; }

			if (T < Period_Rectangular/2) { return Upper_setpoint; }
			else { return Lower_setpoint; }
}
