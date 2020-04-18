/*
 * dc_motor_api.h
 *
 *  Created on: 10.04.2020
 *      Author: Pedro Alonso Sanchez Cruz
 */

#ifndef INC_DC_MOTOR_API_H_
#define INC_DC_MOTOR_API_H_

#define DC_MOTOR_CW_PIN 			GPIO_PIN_4
#define DC_MOTOR_CCW_PIN			GPIO_PIN_5

typedef struct DCMotor_S
{
	TIM_HandleTypeDef 	*motor_pwm_ctrl;
	uint16_t			cw_gpio_pin;
	uint16_t			ccw_gpio_pin;
	uint16_t 			pwm_value;
	uint8_t				direction_flag;
} DCMotor_S;

typedef enum DCMotorDirection_E
{
	MOTOR_SPIN_CW,
	MOTOR_SPIN_CCW,
	MOTOR_SPIN_STOP,
} DCMotorDirection_E;

void DCMotorInit(DCMotor_S *dc_motor_, TIM_HandleTypeDef *timer_handler_);
void DCMotorRPMSet(DCMotor_S *dc_motor_);
void DCMotorRPMStop(DCMotor_S *dc_motor_);

#endif /* INC_DC_MOTOR_API_H_ */
