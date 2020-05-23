/*
 * buzzer_api.c
 *
 *  Created on: Apr 16, 2020
 *      Author: Pedro Alonso Sanchez Cruz
 */

#include "buzzer_api.h"

void BuzzerInit(Buzzer_S *buzzer_)
{
	buzzer_->cycle_frequency_hz 		= DEFAULT_BUZZER_CYCLE_TIME_MS;
	buzzer_->sound_volume_percentage 	= DEFAULT_BUZZER_DUTY_CYCLE;
	buzzer_->sound_frequency_hz 		= DEFAULT_BUZZER_PERIOD_MS;
}

void BuzzerUpdateParams(Buzzer_S *buzzer_, uint16_t sound_frequency_, float duty_cycle_, uint16_t alarm_frequency_)
{
	buzzer_->sound_frequency_hz 		= sound_frequency_;
	buzzer_->sound_volume_percentage 	= duty_cycle_;
	buzzer_->cycle_frequency_hz 		= alarm_frequency_;

	buzzer_->timer->Instance = TIM10;
	buzzer_->timer->Init.Prescaler = 0;
	buzzer_->timer->Init.CounterMode = TIM_COUNTERMODE_UP;
	buzzer_->timer->Init.Period = (uint32_t) (SYS_CLOCK_FREQUENCY / buzzer_->sound_frequency_hz);
	buzzer_->timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	buzzer_->timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	if (HAL_TIM_Base_Init(buzzer_->timer) != HAL_OK)
	{
		//Error_Handler();
	}
	if (HAL_TIM_PWM_Init(buzzer_->timer) != HAL_OK)
	{
		//Error_Handler();
	}

	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode 		= TIM_OCMODE_PWM1;
	sConfigOC.Pulse 		= (uint32_t) ((SYS_CLOCK_FREQUENCY / buzzer_->sound_frequency_hz) * buzzer_->sound_volume_percentage);
	sConfigOC.OCPolarity 	= TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity 	= TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode 	= TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState 	= TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState 	= TIM_OCNIDLESTATE_RESET;

	if (HAL_TIM_PWM_ConfigChannel(buzzer_->timer, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		//Error_Handler();
	}

	HAL_TIM_PWM_Start(buzzer_->timer, TIM_CHANNEL_1);
}
