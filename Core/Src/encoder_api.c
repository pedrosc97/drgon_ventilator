/*
 * sd_encoder_api.c
 *
 *  Created on: Apr 12, 2020
 *      Author: pedro
 */

#include "encoder_api.h"

void EncoderInit(Encoder_S *encoder_, EncoderModel_E model_)
{
	encoder_->model = model_;
	encoder_->rpm = 0;
	encoder_->prev_pulse_count = 0;
}

void UpdateEncoderParams(Encoder_S *encoder_, uint32_t encoder_timer_, uint16_t timestep_ms_)
{
	int32_t delta_count;
	delta_count = encoder_timer_ - encoder_->prev_pulse_count;
	encoder_->prev_pulse_count = (uint16_t) encoder_timer_;

	float pulses_per_second;
	pulses_per_second = ((float) delta_count / (float)timestep_ms_) * (float) MILISECONDS_PER_SECOND;

	float revs_per_minute;
	revs_per_minute = (pulses_per_second / (float) encoder_->model) * (float) SECONDS_PER_MINUTE;

	if (revs_per_minute < RPM_LPF_THRESHOLD && revs_per_minute > -RPM_LPF_THRESHOLD)
	{
		encoder_->rpm = revs_per_minute;
	}
}
