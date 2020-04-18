/*
 * ventilator_api.h
 *
 *  Created on: 17.04.2020
 *      Author: Pedro Alonso Sánchez Cruz
 */

#ifndef INC_VENTILATOR_API_H_
#define INC_VENTILATOR_API_H_

typedef struct Ventilator_S
{
	uint8_t		i_e_ratio; // 1 to 4
	uint8_t		respiration_frequency; // 8 to 40 Hz
	uint16_t	tidal_volume; // 0 to 800 ml in one increment
	uint8_t		pressure_level_alarm_value; // 0 to 80 in 5 increments

	volatile uint8_t	config_flags;	// 0000 0[calibration_btn][silence_alarms][enable_routine]
	volatile uint8_t	alarm_flags;	// 0000 0000

} Ventilator_S;

#endif /* INC_VENTILATOR_API_H_ */
