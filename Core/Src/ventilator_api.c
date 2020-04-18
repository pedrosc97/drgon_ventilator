/*
 * ventilator_api.c
 *
 *  Created on: 17.04.2020
 *      Author: Pedro Alonso Sánchez Cruz
 */

#include "stm32f1xx_hal.h"
#include "ventilator_api.h"

void VentilatorInit(Ventilator_S *ventilator_)
{
	ventilator_->status_flags = DISABLE_ALL;
	ventilator_->alarm_flags = DISABLE_ALL;
}

void UpdateVentilatorParams(Ventilator_S *ventilator_, Potentiometer_S *potentiometer_)
{
	if (potentiometer_[I_E_RATIO_CONTROL].value < 1024)
	{
		ventilator_->i_e_ratio = 1;
	}
	else if (potentiometer_[I_E_RATIO_CONTROL].value < 2048)
	{
		ventilator_->i_e_ratio = 2;
	}
	else if (potentiometer_[I_E_RATIO_CONTROL].value < 3072)
	{
		ventilator_->i_e_ratio = 3;
	}
	else
	{
		ventilator_->i_e_ratio = 4;
	}

	ventilator_->tidal_volume 				= (int) (((TIDAL_VOLUME_MAX_VAL) / ADC_MAX_VAL) * (float)(potentiometer_[TIDAL_VOLUME_CONTROL].value));
	ventilator_->respiration_frequency 		= (int) (((RESPIRATION_FREQ_MAX_VAL - RESPIRATION_FREQ_MIN_VAL) / ADC_MAX_VAL) * (float)(potentiometer_[RESPIRATORY_FREQUENCY_CONTROL].value) + RESPIRATION_FREQ_MIN_VAL);
	ventilator_->pressure_level_alarm_value = (int) (((PRESSURE_LEVEL_ALARM_MAX_VAL - PRESSURE_LEVEL_ALARM_MIN_VAL) / ADC_MAX_VAL) * (float)(potentiometer_[PRESSURE_VALUE_CONTROL].value) + PRESSURE_LEVEL_ALARM_MIN_VAL);

	ventilator_->respiration_period_ms		= (int) ((60000.0f / (float) ventilator_->respiration_frequency));
	ventilator_->inspiration_period_ms		= (int) ((1.0 / ((float) ventilator_->i_e_ratio + 1.0)) * ventilator_->respiration_period_ms);
	ventilator_->exhalation_period_ms		= (int) ((float) ventilator_->i_e_ratio * ventilator_->inspiration_period_ms);
}

void ToggleRoutineEnaParam(Ventilator_S *ventilator_)
{
	if ((ventilator_->status_flags & ENABLE_ROUTINE) == 0)
	{
		ventilator_->status_flags = ventilator_->status_flags | ENABLE_ROUTINE;
	}
	else
	{
		ventilator_->status_flags = ventilator_->status_flags & DISABLE_ROUTINE;
	}
}

void ToggleSilenceAlarmParam(Ventilator_S *ventilator_)
{
	if ((ventilator_->status_flags & SILENCE_ALARMS) == 0)
	{
		ventilator_->status_flags = ventilator_->status_flags | SILENCE_ALARMS;
	}
	else
	{
		ventilator_->status_flags = ventilator_->status_flags & ENABLE_ALARMS;
	}
}

void ToggleCalibrationParam(Ventilator_S *ventilator_)
{
	if ((ventilator_->status_flags & START_CALIBRATION) == 0)
	{
		ventilator_->status_flags = ventilator_->status_flags | START_CALIBRATION;
	}
	else
	{
		ventilator_->status_flags = ventilator_->status_flags & STOP_CALIBRATION;
	}
}
