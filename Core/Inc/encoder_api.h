/*
 * hd_encoder_api.h
 *
 *  Created on: 10.04.2020
 *      Author: Pedro Alonso Sanchez Cruz
 */

#ifndef INC_ENCODER_API_H_
#define INC_ENCODER_API_H_

#define ARM_ENCODER_PULSE_PER_REV	2400
#define MOTOR_ENCODER_PULSE_PER_REV	1600

#define OVERLOADED_16_BIT		0xFFFF
#define MILISECONDS_PER_SECOND	1000.0f
#define SECONDS_PER_MINUTE		60.0f
#define RPM_LPF_THRESHOLD		150.0f
#define DEGREES_PER_REV			360.0f

#include "stm32f4xx_hal.h"

typedef enum EncoderModel_E
{
	ARM_MODEL = ARM_ENCODER_PULSE_PER_REV,
	MOTOR_MODEL = MOTOR_ENCODER_PULSE_PER_REV,
} EncoderModel_E;

typedef struct Encoder_S
{
	float				rpm;
	float				angle;
	EncoderModel_E		model;
	uint16_t			prev_pulse_count;
	TIM_TypeDef 		*timer;
} Encoder_S;

void EncoderInit(Encoder_S *encoder_, TIM_TypeDef *timer_, EncoderModel_E model_);
void UpdateEncoderParams(Encoder_S *encoder_, uint16_t timestep_ms_);

#endif /* INC_ENCODER_API_H_ */
