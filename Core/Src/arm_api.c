/*
 * arm_api.c
 *
 *  Created on: 19.04.2020
 *      Author: pedro
 */

#include "arm_api.h"

void ArmInit(Arm_S *arm_, TIM_HandleTypeDef *motor_timer_)
{
	arm_->arm_rpm = 0;
	arm_->current_angle = 0;
	arm_->prev_angle = 0;
	arm_->current_omega = 0;
	arm_->prev_omega = 0;

	DCMotorInit(&arm_->dc_motor, motor_timer_);
}

void ArmSetRPM(Arm_S *arm_, float *rpm_)
{
	arm_->arm_rpm = *rpm_;
	arm_->dc_motor.pwm_value = (DC_MOTOR_PWM_RPM_RATIO / ARM_MOTOR_REV_RATIO) * *rpm_;
	if (*rpm_ < 0)
	{
		arm_->dc_motor.direction_flag = MOTOR_SPIN_CCW;
	}
	else if (*rpm_ == 0)
	{
		arm_->dc_motor.direction_flag = MOTOR_SPIN_STOP;
	}
	else
	{
		arm_->dc_motor.direction_flag = MOTOR_SPIN_CW;
	}
	DCMotorRPMSet(&arm_->dc_motor);
}

void ArmSetAngSpeed(Arm_S *arm_, float *omega_)
{
	float rpm = (*omega_ * 60.0f) / (2.0f * PI);
	ArmSetRPM(arm_, &rpm);
}

void ArmMoveDeltaAngle(Arm_S *arm_, float delta_theta_, float delta_time_)
{
	float omega = ((delta_theta_ * PI) / 180) / delta_time_;
	ArmSetAngSpeed(arm_, &omega);
}
