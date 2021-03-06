/*
 * dc_motor_api.c
 *
 *  Created on: Apr 10, 2020
 *      Author: Pedro Alonso Sanchez Cruz
 */

#include "stm32f1xx_hal.h"
#include "dc_motor_api.h"
#include "main.h"

void DCMotorInit(DCMotor_S *dc_motor_, TIM_HandleTypeDef *timer_handler_)
{
	dc_motor_->motor_pwm_ctrl = timer_handler_;
	dc_motor_->pwm_value = 0;
	dc_motor_->direction_flag = MOTOR_SPIN_CW;
}

void DCMotorRPMSet(DCMotor_S *dc_motor_)
{
	TIM_OC_InitTypeDef sConfigOC;

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = dc_motor_->pwm_value - 1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    HAL_TIM_PWM_Stop(dc_motor_->motor_pwm_ctrl, TIM_CHANNEL_1);

    if (dc_motor_->direction_flag == MOTOR_SPIN_CW)
    {
		HAL_GPIO_WritePin(GPIOB, MotorCW_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, MotorCCW_Pin, GPIO_PIN_RESET);
    }
	else if (dc_motor_->direction_flag == MOTOR_SPIN_CCW)
	{
			HAL_GPIO_WritePin(GPIOB, MotorCW_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, MotorCCW_Pin, GPIO_PIN_SET);
    }
	else
	{
			HAL_GPIO_WritePin(GPIOB, MotorCW_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, MotorCCW_Pin, GPIO_PIN_RESET);
    }

    if (HAL_TIM_PWM_ConfigChannel(dc_motor_->motor_pwm_ctrl, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
    	//Error_Handler();
    }

    HAL_TIM_PWM_Start(dc_motor_->motor_pwm_ctrl, TIM_CHANNEL_1);
}

void DCMotorRPMStop(DCMotor_S *dc_motor_)
{
    HAL_TIM_PWM_Stop(dc_motor_->motor_pwm_ctrl, TIM_CHANNEL_1);
    HAL_GPIO_WritePin(GPIOB, dc_motor_->cw_gpio_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, dc_motor_->ccw_gpio_pin, GPIO_PIN_RESET);
}
