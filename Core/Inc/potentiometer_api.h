/*
 * potentiometer_api.h
 *
 *  Created on: 13.04.2020
 *      Author: Pedro Alonso Sanchez Cruz
 */

#ifndef INC_POTENTIOMETER_API_H_
#define INC_POTENTIOMETER_API_H_

#include "stm32f4xx_hal.h"

typedef enum PotControls_E
{
	TIDAL_VOLUME_CONTROL, 			// PA0
	I_E_RATIO_CONTROL, 				// PA1
	RESPIRATORY_FREQUENCY_CONTROL, 	// PA2
	PRESSURE_VALUE_CONTROL, 		// PA3
									// Add more controls here in order of Pinout
	TOTAL_CONTROLS_COUNT, 			// Always leave this at the bottom
} PotControls_E;

typedef enum ADCRank_E
{
	PIN_A2,
	PIN_A3,
	PIN_A0,
	PIN_A1,
} ADCRank_E;

typedef volatile struct Potentiometer_S
{
	ADCRank_E		rank;
	uint16_t		value;
} Potentiometer_S;

void PotControlsInit(Potentiometer_S *pot_array_);
void PotControlsValueUpdate(Potentiometer_S *pot_array_, uint16_t *adc_values_);

#endif /* INC_POTENTIOMETER_API_H_ */
