/*
 * FA_utilities.c
 *
 *  Created on: Apr 10, 2024
 *      Author: Maximilien Kulbicki
 */

/* Includes */
#include "FA_utilities.h"

#include "lcd.h"

/* Functions */

/*
 * Converts the value of the desired pulse width (us)
 * into a counter value and sets it into the timer 4.
 */
void set_pwm_tim4(uint16_t pulse_prd){
	uint16_t period = 10000; // us
	uint16_t counter_ref = pulse_prd * 4096 / period;
	TIM4->CCR1 = counter_ref;
}

/*
 * Get a new value of the ADC and compute a new
 * angle value. Used for the 2 potentiometer.
 */
void get_new_pot_values(ADC_HandleTypeDef *hadc, float *adc_value){

	HAL_ADC_Start(hadc);
	if (HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY) == HAL_OK) {
		adc_value[0] = HAL_ADC_GetValue(hadc);
		HAL_Delay(1);
	}

	if (HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY) == HAL_OK) {
		adc_value[1] = HAL_ADC_GetValue(hadc);
		HAL_Delay(1);
	}
	HAL_ADC_Stop(hadc);

	pot_value_to_rad(adc_value);
}

/*
 * Converts the ADC values to an angle
 * in radian.
 */
void pot_value_to_rad(float *adc_value){

	// ADC -> Angle
	// adc_value = adc_value[0] (value pot box) & adc_value[1] (value pot arm)

}
