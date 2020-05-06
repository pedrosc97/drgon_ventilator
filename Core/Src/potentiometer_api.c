/*
 * potentiometer_api.c
 *
 *  Created on: Apr 13, 2020
 *      Author: Pedro Alonso Sanchez Cruz
 */

#include "potentiometer_api.h"

void PotControlsInit(volatile Potentiometer_S *pot_array_)
{
	pot_array_[TIDAL_VOLUME_CONTROL].rank 				= PIN_A0;
	pot_array_[I_E_RATIO_CONTROL].rank 					= PIN_A1;
	pot_array_[RESPIRATORY_FREQUENCY_CONTROL].rank 		= PIN_A2;
	pot_array_[PRESSURE_VALUE_CONTROL].rank 			= PIN_A3;
}

void PotControlsValueUpdate(volatile Potentiometer_S *pot_array_, uint16_t *adc_values_)
{
	pot_array_[TIDAL_VOLUME_CONTROL].value 				= adc_values_[pot_array_[TIDAL_VOLUME_CONTROL].rank];
	pot_array_[I_E_RATIO_CONTROL].value 				= adc_values_[pot_array_[I_E_RATIO_CONTROL].rank];
	pot_array_[RESPIRATORY_FREQUENCY_CONTROL].value 	= adc_values_[pot_array_[RESPIRATORY_FREQUENCY_CONTROL].rank];
	pot_array_[PRESSURE_VALUE_CONTROL].value 			= adc_values_[pot_array_[PRESSURE_VALUE_CONTROL].rank];
}
