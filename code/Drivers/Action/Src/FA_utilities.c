/*
 * FA_utilities.c
 *
 *  Created on: Apr 10, 2024
 *      Author: Maximilien Kulbicki
 */

/* Includes */
#include "FA_utilities.h"

/* Functions */

/*
 * Converts the value of the desired pulse width (us)
 * into a counter value and sets it into the timer 4.
 */
void set_pwm_tim4(uint16_t pulse_prd){
	uint16_t period = 10000; // us
	uint16_t counter_ref = pulse_prd * 4096 / period;
	TIM4->CCR1 = counter_ref;
};


