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
#include "stm32f1xx_hal.h"

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
#define PrsSnsrCSADCOut_Pin GPIO_PIN_14
#define PrsSnsrCSADCOut_GPIO_Port GPIOC
#define PrsSnsrCSEEOut_Pin GPIO_PIN_15
#define PrsSnsrCSEEOut_GPIO_Port GPIOC
#define SYS_ExternOscIn_Pin GPIO_PIN_0
#define SYS_ExternOscIn_GPIO_Port GPIOD
#define SYS_ExternOscOut_Pin GPIO_PIN_1
#define SYS_ExternOscOut_GPIO_Port GPIOD
#define VolumePotIn_Pin GPIO_PIN_0
#define VolumePotIn_GPIO_Port GPIOA
#define IERatioPotIn_Pin GPIO_PIN_1
#define IERatioPotIn_GPIO_Port GPIOA
#define FrequencyPotIn_Pin GPIO_PIN_2
#define FrequencyPotIn_GPIO_Port GPIOA
#define PressureLvlPotIn_Pin GPIO_PIN_3
#define PressureLvlPotIn_GPIO_Port GPIOA
#define PressureSensorIn_Pin GPIO_PIN_4
#define PressureSensorIn_GPIO_Port GPIOA
#define ADCPlaceholderIn_Pin GPIO_PIN_5
#define ADCPlaceholderIn_GPIO_Port GPIOA
#define MotorEncoderA_Pin GPIO_PIN_6
#define MotorEncoderA_GPIO_Port GPIOA
#define MotorEncoderB_Pin GPIO_PIN_7
#define MotorEncoderB_GPIO_Port GPIOA
#define BuzzerOut_Pin GPIO_PIN_0
#define BuzzerOut_GPIO_Port GPIOB
#define LCD_SCL_Pin GPIO_PIN_10
#define LCD_SCL_GPIO_Port GPIOB
#define LCD_SDA_Pin GPIO_PIN_11
#define LCD_SDA_GPIO_Port GPIOB
#define EditBtnIn_Pin GPIO_PIN_12
#define EditBtnIn_GPIO_Port GPIOB
#define EditBtnIn_EXTI_IRQn EXTI15_10_IRQn
#define CalibrationBtnIn_Pin GPIO_PIN_13
#define CalibrationBtnIn_GPIO_Port GPIOB
#define CalibrationBtnIn_EXTI_IRQn EXTI15_10_IRQn
#define AlarmSilenceBtnIn_Pin GPIO_PIN_14
#define AlarmSilenceBtnIn_GPIO_Port GPIOB
#define AlarmSilenceBtnIn_EXTI_IRQn EXTI15_10_IRQn
#define MotorPWMOut_Pin GPIO_PIN_8
#define MotorPWMOut_GPIO_Port GPIOA
#define ModeLEDOut_Pin GPIO_PIN_9
#define ModeLEDOut_GPIO_Port GPIOA
#define PowerOnLEDOut_Pin GPIO_PIN_10
#define PowerOnLEDOut_GPIO_Port GPIOA
#define StartStopBtnIn_Pin GPIO_PIN_11
#define StartStopBtnIn_GPIO_Port GPIOA
#define StartStopBtnIn_EXTI_IRQn EXTI15_10_IRQn
#define SYS_DebugSWDIO_Pin GPIO_PIN_13
#define SYS_DebugSWDIO_GPIO_Port GPIOA
#define SYS_DebugSWCLK_Pin GPIO_PIN_14
#define SYS_DebugSWCLK_GPIO_Port GPIOA
#define ArmEncoderA_Pin GPIO_PIN_6
#define ArmEncoderA_GPIO_Port GPIOB
#define ArmEncoderB_Pin GPIO_PIN_7
#define ArmEncoderB_GPIO_Port GPIOB
#define MotorCW_Pin GPIO_PIN_8
#define MotorCW_GPIO_Port GPIOB
#define MotorCCW_Pin GPIO_PIN_9
#define MotorCCW_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
