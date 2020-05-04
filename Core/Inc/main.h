/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AlarmLED3_Pin GPIO_PIN_2
#define AlarmLED3_GPIO_Port GPIOE
#define AlarmLED4_Pin GPIO_PIN_3
#define AlarmLED4_GPIO_Port GPIOE
#define AlarmLED5_Pin GPIO_PIN_4
#define AlarmLED5_GPIO_Port GPIOE
#define AlarmLED6_Pin GPIO_PIN_5
#define AlarmLED6_GPIO_Port GPIOE
#define AlarmLED7_Pin GPIO_PIN_6
#define AlarmLED7_GPIO_Port GPIOE
#define AlarmLED8_Pin GPIO_PIN_0
#define AlarmLED8_GPIO_Port GPIOC
#define SysOnLED_Pin GPIO_PIN_1
#define SysOnLED_GPIO_Port GPIOC
#define MotorPWMOut_Pin GPIO_PIN_1
#define MotorPWMOut_GPIO_Port GPIOA
#define PrsrSnsrDRDY_Pin GPIO_PIN_4
#define PrsrSnsrDRDY_GPIO_Port GPIOA
#define VolumePotIn_Pin GPIO_PIN_6
#define VolumePotIn_GPIO_Port GPIOA
#define IERatioPotIn_Pin GPIO_PIN_7
#define IERatioPotIn_GPIO_Port GPIOA
#define FrequencyPotIn_Pin GPIO_PIN_0
#define FrequencyPotIn_GPIO_Port GPIOB
#define PressureLvlPotIn_Pin GPIO_PIN_1
#define PressureLvlPotIn_GPIO_Port GPIOB
#define StartStopBtnIn_Pin GPIO_PIN_12
#define StartStopBtnIn_GPIO_Port GPIOE
#define StartStopBtnIn_EXTI_IRQn EXTI15_10_IRQn
#define EditBtnIn_Pin GPIO_PIN_13
#define EditBtnIn_GPIO_Port GPIOE
#define EditBtnIn_EXTI_IRQn EXTI15_10_IRQn
#define CalibrationBtnIn_Pin GPIO_PIN_14
#define CalibrationBtnIn_GPIO_Port GPIOE
#define CalibrationBtnIn_EXTI_IRQn EXTI15_10_IRQn
#define AlarmSilenceBtnIn_Pin GPIO_PIN_15
#define AlarmSilenceBtnIn_GPIO_Port GPIOE
#define AlarmSilenceBtnIn_EXTI_IRQn EXTI15_10_IRQn
#define ArmEncoderA_Pin GPIO_PIN_12
#define ArmEncoderA_GPIO_Port GPIOD
#define ArmEncoderB_Pin GPIO_PIN_13
#define ArmEncoderB_GPIO_Port GPIOD
#define MotorEncoderA_Pin GPIO_PIN_6
#define MotorEncoderA_GPIO_Port GPIOC
#define MotorEncoderB_Pin GPIO_PIN_7
#define MotorEncoderB_GPIO_Port GPIOC
#define MotorCW_Pin GPIO_PIN_4
#define MotorCW_GPIO_Port GPIOD
#define MotorCCW_Pin GPIO_PIN_5
#define MotorCCW_GPIO_Port GPIOD
#define PSnsrCSADCOut_Pin GPIO_PIN_6
#define PSnsrCSADCOut_GPIO_Port GPIOD
#define PSnsrCSEEOut_Pin GPIO_PIN_7
#define PSnsrCSEEOut_GPIO_Port GPIOD
#define BuzzerPWMOut_Pin GPIO_PIN_8
#define BuzzerPWMOut_GPIO_Port GPIOB
#define AlarmLED1_Pin GPIO_PIN_0
#define AlarmLED1_GPIO_Port GPIOE
#define AlarmLED2_Pin GPIO_PIN_1
#define AlarmLED2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
