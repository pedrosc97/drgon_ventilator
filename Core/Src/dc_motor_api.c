/*
 * dc_motor_api.c
 *
 *  Created on: Apr 10, 2020
 *      Author: Pedro Alonso Sanchez Cruz
 */

#include "stm32f1xx_hal.h"
#include "dc_motor_api.h"

void DCMotorInit(DCMotor_S *dc_motor_, TIM_HandleTypeDef *timer_handler_)
{
	dc_motor_->motor_pwm_ctrl = timer_handler_;
}

void DCMotorRPMSet(DCMotor_S *dc_motor_, uint16_t rpm_, DCMotorDirection_E direction_)
{
    TIM_OC_InitTypeDef sConfigOC;

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = rpm_ - 1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    HAL_TIM_PWM_Stop(dc_motor_->motor_pwm_ctrl, TIM_CHANNEL_1);

    if (direction_ == MOTOR_SPIN_CW)
    {
		HAL_GPIO_WritePin(GPIOB, DC_MOTOR_CW_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, DC_MOTOR_CCW_PIN, GPIO_PIN_RESET);
    }
	else if (direction_ == MOTOR_SPIN_CCW)
	{
			HAL_GPIO_WritePin(GPIOB, DC_MOTOR_CW_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, DC_MOTOR_CCW_PIN, GPIO_PIN_SET);
    }
	else
	{
			HAL_GPIO_WritePin(GPIOB, DC_MOTOR_CW_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, DC_MOTOR_CCW_PIN, GPIO_PIN_RESET);
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
