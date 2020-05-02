/*
 * hd_encoder_api.h
 *
 *  Created on: 10.04.2020
 *      Author: Pedro Alonso Sanchez Cruz
 */

#ifndef INC_ENCODER_API_H_
#define INC_ENCODER_API_H_

#define OVERLOADED_16_BIT		0xFFFF
#define MILISECONDS_PER_SECOND	1000
#define SECONDS_PER_MINUTE		60
#define RPM_LPF_THRESHOLD		150

#include "stm32f4xx_hal.h"

typedef enum EncoderModel_E
{
	HD_MODEL = 600,
	SD_MODEL = 80,
} EncoderModel_E;

typedef struct Encoder_S
{
	float				rpm;
	EncoderModel_E		model;
	uint16_t			prev_pulse_count;
} Encoder_S;

void EncoderInit(Encoder_S *encoder_, EncoderModel_E model_);
void UpdateEncoderParams(Encoder_S *encoder_, uint32_t encoder_timer_, uint16_t timestep_ms_);

#endif /* INC_ENCODER_API_H_ */
