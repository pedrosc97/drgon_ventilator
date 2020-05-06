/*
 * buzzer_api.h
 *
 *  Created on: Apr 16, 2020
 *      Author: Pedro Alonso Sánchez Cruz
 */

#ifndef INC_BUZZER_API_H_
#define INC_BUZZER_API_H_

#include "stm32f4xx_hal.h"

#define DEFAULT_BUZZER_DUTY_CYCLE		0.5
#define DEFAULT_BUZZER_PERIOD_MS		1000
#define DEFAULT_BUZZER_CYCLE_TIME_MS	2
#define SYS_CLOCK_FREQUENCY				80000000.0f

typedef struct Buzzer_S
{
	TIM_HandleTypeDef	*timer;
	uint16_t			sound_frequency_hz;
	uint16_t			cycle_frequency_hz;
	uint8_t				sound_volume_percentage;
} Buzzer_S;

void BuzzerInit(Buzzer_S *buzzer_);
void BuzzerUpdateParams(Buzzer_S *buzzer_, uint16_t sound_frequency_, float duty_cycle_, uint16_t alarm_frequency_);

#endif /* INC_BUZZER_API_H_ */
