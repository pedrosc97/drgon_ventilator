/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "dc_motor_api.h"
#include "lcd_display_api.h"
#include "encoder_api.h"
#include "potentiometer_api.h"
#include "buzzer_api.h"
#include "ventilator_api.h"
#include "arm_api.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* Configuration for the DC Motor test routine */

/* Task refresh rate configuration */
#define MOTOR_RPM_CALCULATE_TIMESTEP_MS			10
#define ARM_RPM_CALCULATE_TIMESTEP_MS			10
#define LCD_DISPLAY_UPDATE_TIMESTEP_MS			100
#define ALARM_UPDATE_TIMESTEP_MS				10
#define ALARM_SILENCE_TIMEOUT_MS				2000

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

osThreadId mainRoutineHandle;
osThreadId displayUpdateHandle;
osThreadId motorEncoderHandle;
osThreadId initTaskHandle;
osThreadId alarmsTaskHandle;
osThreadId updatePotsHandle;
osThreadId calibRoutineHandle;
osThreadId armEncoderHandle;
osThreadId diagnosticsTaskHandle;
/* USER CODE BEGIN PV */

DCMotor_S 			dc_motor;
LCDDisplay_S 		lcd_display;
Encoder_S			motor_encoder;
Buzzer_S			buzzer;
Potentiometer_S		pot_controls_a[TOTAL_CONTROLS_COUNT];
Ventilator_S		ventilator;

uint32_t			prev_systick = 0;
volatile uint8_t 	unwind_flag = 0;
/* Debugging Variables */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
void startMainRoutine(void const * argument);
void startDisplayUpdate(void const * argument);
void startMotorEncoder(void const * argument);
void startInitTask(void const * argument);
void startAlarmsTask(void const * argument);
void startUpdatePots(void const * argument);
void startCalibRoutine(void const * argument);
void startArmEncoder(void const * argument);
void startDiagnostics(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if (GPIO_Pin == EditBtnIn_Pin)
	{
		osThreadResume(updatePotsHandle);
	}

	if (GPIO_Pin == CalibrationBtnIn_Pin)
	{
		uint32_t	WakeTime = osKernelSysTick();

		if ((WakeTime - prev_systick) > 500)
		{
			ToggleCalibrationParam(&ventilator);
			prev_systick = WakeTime;
			osThreadResume(calibRoutineHandle);
		}
	}

	if (GPIO_Pin == AlarmSilenceBtnIn_Pin)
	{
		uint32_t	WakeTime = osKernelSysTick();
		if ((WakeTime - prev_systick) > 500)
		{
			//ToggleSilenceAlarmParam(&ventilator);
			if (unwind_flag == 0)
			{
				unwind_flag = 1;
			}
			else if (unwind_flag == 1)
			{
				unwind_flag = 2;
			}
			else
			{
				unwind_flag = 0;
			}
			prev_systick = WakeTime;
		}
	}

	if (GPIO_Pin == StartStopBtnIn_Pin)
	{
		uint32_t	WakeTime = osKernelSysTick();
		if ((WakeTime - prev_systick) > 500)
		{
			ToggleRoutineEnaParam(&ventilator);
			prev_systick = WakeTime;
		}
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  DCMotorInit(&dc_motor, &htim1);
  EncoderInit(&motor_encoder, HD_MODEL);
  LCDInit(&lcd_display, &hi2c2);
  PotControlsInit(pot_controls_a);
  BuzzerInit(&buzzer);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of mainRoutine */
  osThreadDef(mainRoutine, startMainRoutine, osPriorityRealtime, 0, 128);
  mainRoutineHandle = osThreadCreate(osThread(mainRoutine), NULL);

  /* definition and creation of displayUpdate */
  osThreadDef(displayUpdate, startDisplayUpdate, osPriorityNormal, 0, 128);
  displayUpdateHandle = osThreadCreate(osThread(displayUpdate), NULL);

  /* definition and creation of motorEncoder */
  osThreadDef(motorEncoder, startMotorEncoder, osPriorityAboveNormal, 0, 128);
  motorEncoderHandle = osThreadCreate(osThread(motorEncoder), NULL);

  /* definition and creation of initTask */
  osThreadDef(initTask, startInitTask, osPriorityRealtime, 0, 128);
  initTaskHandle = osThreadCreate(osThread(initTask), NULL);

  /* definition and creation of alarmsTask */
  osThreadDef(alarmsTask, startAlarmsTask, osPriorityAboveNormal, 0, 128);
  alarmsTaskHandle = osThreadCreate(osThread(alarmsTask), NULL);

  /* definition and creation of updatePots */
  osThreadDef(updatePots, startUpdatePots, osPriorityRealtime, 0, 128);
  updatePotsHandle = osThreadCreate(osThread(updatePots), NULL);

  /* definition and creation of calibRoutine */
  osThreadDef(calibRoutine, startCalibRoutine, osPriorityRealtime, 0, 128);
  calibRoutineHandle = osThreadCreate(osThread(calibRoutine), NULL);

  /* definition and creation of armEncoder */
  osThreadDef(armEncoder, startArmEncoder, osPriorityAboveNormal, 0, 128);
  armEncoderHandle = osThreadCreate(osThread(armEncoder), NULL);

  /* definition and creation of diagnosticsTask */
  osThreadDef(diagnosticsTask, startDiagnostics, osPriorityAboveNormal, 0, 128);
  diagnosticsTaskHandle = osThreadCreate(osThread(diagnosticsTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4800-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3200-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 2;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 2;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  HAL_TIM_Encoder_Start_IT(&htim3, htim3.Channel);
  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2400-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 2;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 2;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  HAL_TIM_Encoder_Start_IT(&htim4, htim4.Channel);
  /* USER CODE END TIM4_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MotorCW_Pin|MotorCCW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : EditBtnIn_Pin CalibrationBtnIn_Pin AlarmSilenceBtnIn_Pin */
  GPIO_InitStruct.Pin = EditBtnIn_Pin|CalibrationBtnIn_Pin|AlarmSilenceBtnIn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : StartStopBtnIn_Pin */
  GPIO_InitStruct.Pin = StartStopBtnIn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(StartStopBtnIn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MotorCW_Pin MotorCCW_Pin */
  GPIO_InitStruct.Pin = MotorCW_Pin|MotorCCW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_startMainRoutine */
/**
  * @brief  Function implementing the mainRoutine thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_startMainRoutine */
void startMainRoutine(void const * argument)
{
  /* USER CODE BEGIN 5 */
	uint32_t PreviousWakeTime = osKernelSysTick();

  /* Infinite loop */
	for(;;)
	{
		if (((ventilator.status_flags & ENABLE_ROUTINE) == ENABLE_ROUTINE) && !((ventilator.status_flags & START_CALIBRATION) == START_CALIBRATION))
		{
			dc_motor.pwm_value = ventilator.motor_pwm_value_in;
			dc_motor.direction_flag = MOTOR_SPIN_CCW;
			DCMotorRPMSet(&dc_motor);

			//osDelayUntil(&PreviousWakeTime, ventilator.inspiration_period_ms);
			if (TIM4->CNT < (ventilator.end_angle_pulse - 3))
			{
				osDelayUntil(&PreviousWakeTime, 10);
			}

			dc_motor.pwm_value = 0;
			dc_motor.direction_flag = MOTOR_SPIN_STOP;
			DCMotorRPMSet(&dc_motor);
			osDelayUntil(&PreviousWakeTime, 100);

			dc_motor.pwm_value = ventilator.motor_pwm_value_out;
			dc_motor.direction_flag = MOTOR_SPIN_CW;
			DCMotorRPMSet(&dc_motor);
			osDelayUntil(&PreviousWakeTime, ventilator.exhalation_period_ms);

			if (TIM4->CNT > (10))
			{
				osDelayUntil(&PreviousWakeTime, 10);
			}

			dc_motor.pwm_value = 0;
			dc_motor.direction_flag = MOTOR_SPIN_STOP;
			DCMotorRPMSet(&dc_motor);
			osDelayUntil(&PreviousWakeTime, 100);
		}
		else if ((ventilator.status_flags & START_CALIBRATION) == START_CALIBRATION)
		{
			osDelayUntil(&PreviousWakeTime, 10);
		}
		else
		{
			dc_motor.pwm_value = 0;
			dc_motor.direction_flag = MOTOR_SPIN_STOP;
			DCMotorRPMSet(&dc_motor);
			BuzzerUpdateParams(&buzzer, 125, 0.5, 6);
			osDelayUntil(&PreviousWakeTime, 10);
		}
	}
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_startDisplayUpdate */
/**
* @brief Function implementing the displayUpdate thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startDisplayUpdate */
void startDisplayUpdate(void const * argument)
{
  /* USER CODE BEGIN startDisplayUpdate */
	char buffer[32];
	uint32_t PreviousWakeTime = osKernelSysTick();
	osThreadResume(updatePotsHandle);

  /* Infinite loop */
	for(;;)
	{
		LCDSetCursorPos(&lcd_display, 0, 0);
		sprintf(buffer, "I:E %03u  T %06lu", ventilator.i_e_ratio, HAL_GetTick());
		LCDSendString(&lcd_display, buffer);

		LCDSetCursorPos(&lcd_display, 1, 0);
		sprintf(buffer, "VOL %03u  PMOT %04lu", ventilator.tidal_volume, TIM3->CNT);
		LCDSendString(&lcd_display, buffer);

		LCDSetCursorPos(&lcd_display, 2, 0);
		sprintf(buffer, "PRS %03u  PARM %04lu", ventilator.pressure_level_alarm_value, TIM4->CNT);
		LCDSendString(&lcd_display, buffer);

		LCDSetCursorPos(&lcd_display, 3, 0);
		sprintf(buffer, "RFQ %03u", ventilator.respiration_frequency);
		LCDSendString(&lcd_display, buffer);

		osDelayUntil(&PreviousWakeTime, LCD_DISPLAY_UPDATE_TIMESTEP_MS);
	}
  /* USER CODE END startDisplayUpdate */
}

/* USER CODE BEGIN Header_startMotorEncoder */
/**
* @brief Function implementing the motorEncoder thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startMotorEncoder */
void startMotorEncoder(void const * argument)
{
  /* USER CODE BEGIN startMotorEncoder */
	uint32_t PreviousWakeTime = osKernelSysTick();

  /* Infinite loop */
	for(;;)
	{
		/* TODO: Refactor of encoder parameters update function */
		//UpdateEncoderParams(&motor_encoder, TIM3->CNT, MOTOR_RPM_CALCULATE_TIMESTEP_MS);
		osDelayUntil(&PreviousWakeTime, MOTOR_RPM_CALCULATE_TIMESTEP_MS);
	}
  /* USER CODE END startMotorEncoder */
}

/* USER CODE BEGIN Header_startInitTask */
/**
* @brief Function implementing the initTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startInitTask */
void startInitTask(void const * argument)
{
  /* USER CODE BEGIN startInitTask */
	VentilatorInit(&ventilator);
	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	osThreadResume(updatePotsHandle);
	osThreadTerminate(initTaskHandle);
  /* USER CODE END startInitTask */
}

/* USER CODE BEGIN Header_startAlarmsTask */
/**
* @brief Function implementing the alarmsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startAlarmsTask */
void startAlarmsTask(void const * argument)
{
  /* USER CODE BEGIN startAlarmsTask */
	uint32_t PreviousWakeTime = osKernelSysTick();
	//osDelayUntil(&PreviousWakeTime, 1000);

  /* Infinite loop */
	for(;;)
	{
		if (((ventilator.status_flags & ENABLE_ROUTINE) == ENABLE_ROUTINE) && !((ventilator.status_flags & START_CALIBRATION) == START_CALIBRATION))
		{
			if (unwind_flag == 0)
			{
				dc_motor.pwm_value = 0;
				dc_motor.direction_flag = MOTOR_SPIN_STOP;
				DCMotorRPMSet(&dc_motor);
			}
			else if (unwind_flag == 1)
			{
				dc_motor.pwm_value = 750;
				dc_motor.direction_flag = MOTOR_SPIN_CW;
				DCMotorRPMSet(&dc_motor);
			}
			else
			{
				dc_motor.pwm_value = 750;
				dc_motor.direction_flag = MOTOR_SPIN_CCW;
				DCMotorRPMSet(&dc_motor);
			}
		}
		osDelayUntil(&PreviousWakeTime, 10);
	}
  /* USER CODE END startAlarmsTask */
}

/* USER CODE BEGIN Header_startUpdatePots */
/**
* @brief Function implementing the updatePots thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startUpdatePots */
void startUpdatePots(void const * argument)
{
  /* USER CODE BEGIN startUpdatePots */
  /* Infinite loop */
	for(;;)
	{
	  uint16_t adc_values[TOTAL_CONTROLS_COUNT] = {0,0,0,0};
	  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_values, TOTAL_CONTROLS_COUNT);
	  PotControlsValueUpdate(pot_controls_a, adc_values);
	  UpdateVentilatorParams(&ventilator, pot_controls_a);
	  osThreadSuspend(updatePotsHandle);
	}
  /* USER CODE END startUpdatePots */
}

/* USER CODE BEGIN Header_startCalibRoutine */
/**
* @brief Function implementing the calibRoutine thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startCalibRoutine */
void startCalibRoutine(void const * argument)
{
  /* USER CODE BEGIN startCalibRoutine */
	uint32_t	PreviousWakeTime = osKernelSysTick();
	uint8_t 	process_flag = 0;
	uint32_t init_count = 0;
	osThreadSuspend(calibRoutineHandle);

  /* Infinite loop */
	for(;;)
	{

			if (process_flag == 0)
			{
				osThreadSuspend(mainRoutineHandle);
				PreviousWakeTime = osKernelSysTick();
				dc_motor.pwm_value = 1000;
				dc_motor.direction_flag = MOTOR_SPIN_CW;
				DCMotorRPMSet(&dc_motor);
				osDelayUntil(&PreviousWakeTime, 3000);

				dc_motor.direction_flag = MOTOR_SPIN_STOP;
				DCMotorRPMSet(&dc_motor);
				osThreadSuspend(calibRoutineHandle);

				init_count = TIM4->CNT;
				dc_motor.pwm_value = 1000;
				dc_motor.direction_flag = MOTOR_SPIN_CCW;
				DCMotorRPMSet(&dc_motor);
				process_flag = 1;
			}

			if (TIM4->CNT != init_count)
			{
				dc_motor.pwm_value = 0;
				dc_motor.direction_flag = MOTOR_SPIN_STOP;
				DCMotorRPMSet(&dc_motor);
				process_flag = 0;
				TIM3->CNT = 0;
				TIM4->CNT = 0;
				ToggleCalibrationParam(&ventilator);
				osThreadResume(mainRoutineHandle);
				osThreadSuspend(calibRoutineHandle);
			}
			else
			{
				osDelayUntil(&PreviousWakeTime, 10);
			}
	}
  /* USER CODE END startCalibRoutine */
}

/* USER CODE BEGIN Header_startArmEncoder */
/**
* @brief Function implementing the armEncoder thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startArmEncoder */
void startArmEncoder(void const * argument)
{
  /* USER CODE BEGIN startArmEncoder */
	uint32_t PreviousWakeTime = osKernelSysTick();

  /* Infinite loop */
	for(;;)
	{
		/* TODO: Refactor of encoder parameters update function */
		//UpdateEncoderParams(&motor_encoder, TIM4->CNT, ARM_RPM_CALCULATE_TIMESTEP_MS);
		osDelayUntil(&PreviousWakeTime, ARM_RPM_CALCULATE_TIMESTEP_MS);
	}
  /* USER CODE END startArmEncoder */
}

/* USER CODE BEGIN Header_startDiagnostics */
/**
* @brief Function implementing the diagnosticsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startDiagnostics */
void startDiagnostics(void const * argument)
{
  /* USER CODE BEGIN startDiagnostics */
	osThreadSuspend(diagnosticsTaskHandle);
	//char buffer[21];
	//uint32_t PreviousWakeTime = osKernelSysTick();

  /* Infinite loop */
	for(;;)
	{
		/*if ((ventilator.status_flags & ENABLE_ROUTINE) == ENABLE_ROUTINE)
		{
			//sprintf(buffer, "%04lu,%04lu,%04lu,%04u", HAL_GetTick(), TIM3->CNT, TIM4->CNT, dc_motor.pwm_value);
			//HAL_UART_Transmit(&huart1, (uint8_t *) buffer, sizeof(buffer), 100);
		}
		osDelayUntil(&PreviousWakeTime, 10);*/

	}
  /* USER CODE END startDiagnostics */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
