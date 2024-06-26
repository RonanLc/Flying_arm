/*
 * FA_utilities.h
 *
 *  Created on: Apr 10, 2024
 *      Author: kulbi
 */
#include "stdint.h"
#include "stm32f4xx_hal.h"


#ifndef INC_FA_UTILITIES_H_
#define INC_FA_UTILITIES_H_


void set_pwm_tim4(uint16_t pulse_prd);
void get_new_pot_values(ADC_HandleTypeDef *hadc, float *adc_value);
void pot_value_to_rad(float *adc_value);


#endif /* INC_FA_UTILITIES_H_ */
