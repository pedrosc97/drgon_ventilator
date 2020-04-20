/*
 * arm_api.h
 *
 *  Created on: 19.04.2020
 *      Author: pedro
 */

#ifndef INC_ARM_API_H_
#define INC_ARM_API_H_

#include "stm32f1xx_hal.h"
#include "dc_motor_api.h"

#define ARM_MOTOR_REV_RATIO		0.0416f
#define DC_MOTOR_PWM_RPM_RATIO  0.0209f
#define PI						3.14159f

typedef struct Arm_S
{
	DCMotor_S 	dc_motor;
	float		arm_rpm;
	float		current_angle;
	float		prev_angle;
	float		current_omega;
	float		prev_omega;
} Arm_S;

void ArmInit(Arm_S *arm_, TIM_HandleTypeDef *motor_timer_);
void ArmSetRPM(Arm_S *arm_, float *rpm_);
void ArmSetAngSpeed(Arm_S *arm_, float *omega_);
void ArmMoveDeltaAngle(Arm_S *arm_, float delta_theta_, float delta_time_);

#endif /* INC_ARM_API_H_ */
