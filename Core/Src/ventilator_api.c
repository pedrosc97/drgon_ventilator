/*
 * ventilator_api.c
 *
 *  Created on: 17.04.2020
 *      Author: Pedro Alonso Sánchez Cruz
 */

#include "ventilator_api.h"

void VentilatorInit(Ventilator_S *ventilator_)
{
	ventilator_->i_e_ratio 						= I_E_RATIO_1_1;
	ventilator_->respiration_frequency 			= (RespirationFrequency_T) RESPIRATION_FREQ_MIN_VAL;
	ventilator_->pressure_level_alarm_value 	= (PressureValue_T) PRESSURE_LEVEL_ALARM_MIN_VAL;
	ventilator_->tidal_volume 					= (TidalVolume_T) TIDAL_VOLUME_MIN_VAL;

	ventilator_->ventilator_alarms 				= NO_ALARMS;
	ventilator_->ventilator_alarms_config		= REPORT_ALARMS;

	ventilator_->main_state_machine 			= MAIN_STATE_STANDBY;
	ventilator_->setup_state_machine 			= SETUP_STATE_STANDBY;
	ventilator_->calibration_state_machine 		= CALIBRATION_STATE_STOP;
	ventilator_->manual_spin_state_machine 		= MANUAL_SPIN_STATE_STOP;
	ventilator_->respiration_state_machine 		= RESPIRATION_STATE_STANDBY;
	ventilator_->forced_volume_state_machine 	= FORCED_VOLUME_STATE_STANDBY;
	ventilator_->assisted_volume_state_machine 	= ASSISTED_VOLUME_STATE_STANDBY;
}

void UpdateVentilatorParams(Ventilator_S *ventilator_, Potentiometer_S *potentiometer_)
{
	if (potentiometer_[I_E_RATIO_CONTROL].value < (ADC_MAX_VAL / 4))
	{
		ventilator_->i_e_ratio = I_E_RATIO_1_1;
	}
	else if (potentiometer_[I_E_RATIO_CONTROL].value < (ADC_MAX_VAL / 4 * 2))
	{
		ventilator_->i_e_ratio = I_E_RATIO_1_2;
	}
	else if (potentiometer_[I_E_RATIO_CONTROL].value < (ADC_MAX_VAL / 4 * 3))
	{
		ventilator_->i_e_ratio = I_E_RATIO_1_3;
	}
	else
	{
		ventilator_->i_e_ratio = I_E_RATIO_1_4;
	}

	ventilator_->tidal_volume 				= (TidalVolume_T) (((TIDAL_VOLUME_MAX_VAL) / ADC_MAX_VAL) * (float)(potentiometer_[TIDAL_VOLUME_CONTROL].value));
	ventilator_->respiration_frequency 		= (RespirationFrequency_T) (((RESPIRATION_FREQ_MAX_VAL - RESPIRATION_FREQ_MIN_VAL) / ADC_MAX_VAL) * (float)(potentiometer_[RESPIRATORY_FREQUENCY_CONTROL].value) + RESPIRATION_FREQ_MIN_VAL);
	ventilator_->pressure_level_alarm_value = (PressureValue_T) (((PRESSURE_LEVEL_ALARM_MAX_VAL - PRESSURE_LEVEL_ALARM_MIN_VAL) / ADC_MAX_VAL) * (float)(potentiometer_[PRESSURE_VALUE_CONTROL].value) + PRESSURE_LEVEL_ALARM_MIN_VAL);

	ventilator_->respiration_period_ms		= (int) ((60000.0f / (float) ventilator_->respiration_frequency));
	ventilator_->inhalation_period_ms		= (int) ((1.0f / ((float) ventilator_->i_e_ratio + 1.0f)) * ventilator_->respiration_period_ms);
	ventilator_->exhalation_period_ms		= (int) ((float) ventilator_->i_e_ratio * ventilator_->inhalation_period_ms);
	ventilator_->end_angle 					= (((float) ventilator_->tidal_volume) / TIDAL_VOLUME_MAX_VAL * ARM_SWEEP_ANGLE);
}
