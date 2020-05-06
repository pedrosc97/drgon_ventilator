/*
 * buzzer_api.c
 *
 *  Created on: Apr 16, 2020
 *      Author: Pedro Alonso Sanchez Cruz
 */

#include "buzzer_api.h"

void BuzzerInit(Buzzer_S *buzzer_)
{
	buzzer_->cycle_frequency_hz = DEFAULT_BUZZER_CYCLE_TIME_MS;
	buzzer_->sound_volume_percentage = DEFAULT_BUZZER_DUTY_CYCLE;
	buzzer_->sound_frequency_hz = DEFAULT_BUZZER_PERIOD_MS;
}

void BuzzerUpdateParams(Buzzer_S *buzzer_, uint16_t sound_frequency_, float duty_cycle_, uint16_t alarm_frequency_)
{
	buzzer_->sound_frequency_hz = (1 / (float) sound_frequency_) * 1000.0f;
	buzzer_->sound_volume_percentage = duty_cycle_;
	buzzer_->cycle_frequency_hz = (uint16_t) (1 / (float) alarm_frequency_ * 1000.0f);

	/*TIM_OC_InitTypeDef sConfigOC;

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	//sConfigOC.Pulse = ;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

	HAL_TIM_PWM_Stop(dc_motor_->motor_pwm_ctrl, TIM_CHANNEL_1);

	if (HAL_TIM_PWM_ConfigChannel(dc_motor_->motor_pwm_ctrl, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		//Error_Handler();
	}

	HAL_TIM_PWM_Start(dc_motor_->motor_pwm_ctrl, TIM_CHANNEL_1);*/
}
