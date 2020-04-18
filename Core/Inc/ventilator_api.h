/*
 * ventilator_api.h
 *
 *  Created on: 17.04.2020
 *      Author: Pedro Alonso Sánchez Cruz
 */

#ifndef INC_VENTILATOR_API_H_
#define INC_VENTILATOR_API_H_

#include "potentiometer_api.h"

#define ADC_MAX_VAL						0xFFF

#define TIDAL_VOLUME_INCREMENT			1.0f
#define TIDAL_VOLUME_MIN_VAL			0.0f
#define TIDAL_VOLUME_MAX_VAL			800.0f

#define PRESSURE_LEVEL_ALARM_INCREMENT	5.0f
#define PRESSURE_LEVEL_ALARM_MIN_VAL	0.0f
#define PRESSURE_LEVEL_ALARM_MAX_VAL	80.0f

#define RESPIRATION_FREQ_STEP			1.0f
#define RESPIRATION_FREQ_MIN_VAL		8.0f
#define RESPIRATION_FREQ_MAX_VAL		40.0f

typedef enum VentilatorStsCfg_E
{
	ENABLE_ROUTINE = 0x01,
	SILENCE_ALARMS = 0x02,
	START_CALIBRATION = 0x04,
	DISABLE_ROUTINE = 0xFE,
	ENABLE_ALARMS = 0xFD,
	STOP_CALIBRATION = 0xFB,
	ENABLE_ALL = 0x07,
	DISABLE_ALL = 0x00,
} VentilatorStsCfg_E;

typedef struct Ventilator_S
{
	uint8_t		i_e_ratio; 						// 1 to 4
	uint8_t		respiration_frequency; 			// 8 to 40 Hz
	uint16_t	tidal_volume; 					// 0 to 800 ml in one increment
	uint8_t		pressure_level_alarm_value; 	// 0 to 80 in 5 increments

	uint16_t	respiration_period_ms;
	uint16_t	inspiration_period_ms;
	uint16_t	exhalation_period_ms;

	volatile uint8_t	status_flags;	// 0000 0[calibration_btn][silence_alarms][enable_routine]
	volatile uint8_t	alarm_flags;	// 0000 0000

} Ventilator_S;

void VentilatorInit(Ventilator_S *ventilator_);
void UpdateVentilatorParams(Ventilator_S *ventilator_, Potentiometer_S *potentiometer_);
void ToggleRoutineEnaParam(Ventilator_S *ventilator_);
void ToggleSilenceAlarmParam(Ventilator_S *ventilator_);
void ToggleCalibrationParam(Ventilator_S *ventilator_);

#endif /* INC_VENTILATOR_API_H_ */
