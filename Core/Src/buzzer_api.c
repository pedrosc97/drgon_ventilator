/*
 * buzzer_api.c
 *
 *  Created on: Apr 16, 2020
 *      Author: Pedro Alonso Sanchez Cruz
 */
#include "stm32f1xx_hal.h"
#include "buzzer_api.h"

void BuzzerInit(Buzzer_S *buzzer_)
{
	buzzer_->cycle_time = DEFAULT_BUZZER_CYCLE_TIME_MS;
	buzzer_->duty_cycle = DEFAULT_BUZZER_DUTY_CYCLE;
	buzzer_->period = DEFAULT_BUZZER_PERIOD_MS;
}

void BuzzerUpdateParams(Buzzer_S *buzzer_, uint16_t sound_frequency_, float duty_cycle_, uint16_t alarm_frequency_)
{
	buzzer_->cycle_time = (1 / (float) sound_frequency_) * 1000.0;
	buzzer_->duty_cycle = duty_cycle_;
	buzzer_->period = (uint16_t) (1 / (float) alarm_frequency_ * 1000.0);
}
