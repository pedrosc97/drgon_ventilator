/*
 * buzzer_api.h
 *
 *  Created on: Apr 16, 2020
 *      Author: Pedro Alonso Sánchez Cruz
 */

#ifndef INC_BUZZER_API_H_
#define INC_BUZZER_API_H_

#define DEFAULT_BUZZER_DUTY_CYCLE		0.5
#define DEFAULT_BUZZER_PERIOD_MS		1000
#define DEFAULT_BUZZER_CYCLE_TIME_MS	2

typedef struct Buzzer_S
{
	float		duty_cycle;
	float		cycle_time;
	uint16_t	period;
} Buzzer_S;

void BuzzerInit(Buzzer_S *buzzer_);
void BuzzerUpdateParams(Buzzer_S *buzzer_, uint16_t sound_frequency_, float duty_cycle_, uint16_t alarm_frequency_);

#endif /* INC_BUZZER_API_H_ */
