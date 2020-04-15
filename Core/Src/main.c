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
#define MOTOR_MAX_PWM_PULSE					4800
#define MOTOR_MIN_PWM_PULSE					0
#define MOTOR_POSITIVE_PWM_STEP				100
#define MOTOR_NEGATIVE_PWM_STEP				100

/* Task refresh rate configuration */
#define RPM_CALCULATE_TIMESTEP_MS			100
#define LCD_DISPLAY_UPDATE_TIMESTEP_MS		10
#define POT_CONTROLS_UPDATE_TIMESTEP_MS		1000
#define ALARM_UPDATE_TIMESTEP_MS			10
#define ALARM_SILENCE_TIMEOUT_MS			2000

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

osThreadId mainRoutineHandle;
osThreadId displayUpdateHandle;
osThreadId encoderRPMHandle;
osThreadId initTaskHandle;
osThreadId alarmsTaskHandle;
/* USER CODE BEGIN PV */

DCMotor_S 			dc_motor;
LCDDisplay_S 		lcd_display;
Encoder_S			motor_encoder;

volatile Potentiometer_S		pot_controls_a[TOTAL_CONTROLS_COUNT];
volatile uint8_t	enable_routine;
volatile uint16_t	main_routine_update_time_ms;
volatile uint8_t	silence_alarms;

/* Debugging Variables */
uint8_t alarm_flag;
volatile uint8_t calibration_btn = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
void startMainRoutine(void const * argument);
void startDisplayUpdate(void const * argument);
void startEncoderRPM(void const * argument);
void startInitTask(void const * argument);
void startAlarmsTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == EditBtnIn_Pin)
	{
		uint16_t adc_values[TOTAL_CONTROLS_COUNT] = {0,0,0,0};
		HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_values, TOTAL_CONTROLS_COUNT);
		PotControlsValueUpdate(pot_controls_a, adc_values);
	}

	if (GPIO_Pin == CalibrationBtnIn_Pin)
	{
		/* TODO: Calibration Routine Callback Function Implementation */
		if (calibration_btn == 0)
		{
			calibration_btn = 1;
		}
		else
		{
			calibration_btn = 0;
		}
	}

	if (GPIO_Pin == AlarmSilenceBtnIn_Pin)
	{
		silence_alarms = 1;
	}

	if (GPIO_Pin == StartStopBtnIn_Pin)
	{
		if (enable_routine == 0)
		{
			enable_routine = 1;
		}
		else
		{
			enable_routine = 0;
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
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  DCMotorInit(&dc_motor, &htim1);
  EncoderInit(&motor_encoder, SD_MODEL);
  LCDInit(&lcd_display, &hi2c2);
  PotControlsInit(pot_controls_a);

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
  osThreadDef(mainRoutine, startMainRoutine, osPriorityAboveNormal, 0, 128);
  mainRoutineHandle = osThreadCreate(osThread(mainRoutine), NULL);

  /* definition and creation of displayUpdate */
  osThreadDef(displayUpdate, startDisplayUpdate, osPriorityAboveNormal, 0, 128);
  displayUpdateHandle = osThreadCreate(osThread(displayUpdate), NULL);

  /* definition and creation of encoderRPM */
  osThreadDef(encoderRPM, startEncoderRPM, osPriorityAboveNormal, 0, 128);
  encoderRPMHandle = osThreadCreate(osThread(encoderRPM), NULL);

  /* definition and creation of initTask */
  osThreadDef(initTask, startInitTask, osPriorityRealtime, 0, 128);
  initTaskHandle = osThreadCreate(osThread(initTask), NULL);

  /* definition and creation of alarmsTask */
  osThreadDef(alarmsTask, startAlarmsTask, osPriorityIdle, 0, 128);
  alarmsTaskHandle = osThreadCreate(osThread(alarmsTask), NULL);

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
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  htim1.Init.Period = 80-1;
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
  htim3.Init.Period = 80-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
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
  HAL_GPIO_WritePin(GPIOA, ModeLEDOut_Pin|PowerOnLEDOut_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MotorCW_Pin|MotorCCW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : EditBtnIn_Pin CalibrationBtnIn_Pin AlarmSilenceBtnIn_Pin StartStopBtnIn_Pin */
  GPIO_InitStruct.Pin = EditBtnIn_Pin|CalibrationBtnIn_Pin|AlarmSilenceBtnIn_Pin|StartStopBtnIn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ModeLEDOut_Pin PowerOnLEDOut_Pin */
  GPIO_InitStruct.Pin = ModeLEDOut_Pin|PowerOnLEDOut_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MotorCW_Pin MotorCCW_Pin */
  GPIO_InitStruct.Pin = MotorCW_Pin|MotorCCW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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

	int8_t pwm_step = MOTOR_POSITIVE_PWM_STEP;

  /* Infinite loop */
	for(;;)
	{
		if (enable_routine == 1)
		{
			if (dc_motor.pwm_value == MOTOR_MIN_PWM_PULSE)
			{
				pwm_step = MOTOR_POSITIVE_PWM_STEP;
			}
			else if (dc_motor.pwm_value == MOTOR_MAX_PWM_PULSE)
			{
				pwm_step = -MOTOR_NEGATIVE_PWM_STEP;
			}

			dc_motor.pwm_value += pwm_step;

			if (dc_motor.direction_flag == MOTOR_SPIN_CW)
			{
				DCMotorRPMSet(&dc_motor);
				dc_motor.direction_flag = MOTOR_SPIN_CW;
			}
			else
			{
				DCMotorRPMSet(&dc_motor);
				dc_motor.direction_flag = MOTOR_SPIN_CW;
			}
		}

		osDelayUntil(&PreviousWakeTime, main_routine_update_time_ms);
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

  /* Infinite loop */
	for(;;)
	{
		LCDSetCursorPos(&lcd_display, 0, 0);
		sprintf(buffer, "VOL %04u  PWM %04u", pot_controls_a[TIDAL_VOLUME_CONTROL].value, dc_motor.pwm_value);
		LCDSendString(&lcd_display, buffer);

		LCDSetCursorPos(&lcd_display, 1, 0);
		sprintf(buffer, "RFQ %04u  RPM %+04ld", pot_controls_a[RESPIRATORY_FREQUENCY_CONTROL].value, (int32_t) motor_encoder.rpm);
		LCDSendString(&lcd_display, buffer);

		LCDSetCursorPos(&lcd_display, 2, 0);
		sprintf(buffer, "I:E %04u  STS %04u", pot_controls_a[I_E_RATIO_CONTROL].value, enable_routine & calibration_btn);
		LCDSendString(&lcd_display, buffer);

		LCDSetCursorPos(&lcd_display, 3, 0);
		sprintf(buffer, "PRS %04u  SIL %04u", pot_controls_a[PRESSURE_VALUE_CONTROL].value, silence_alarms);
		LCDSendString(&lcd_display, buffer);

		osDelayUntil(&PreviousWakeTime, LCD_DISPLAY_UPDATE_TIMESTEP_MS);
	}
  /* USER CODE END startDisplayUpdate */
}

/* USER CODE BEGIN Header_startEncoderRPM */
/**
* @brief Function implementing the encoderRPM thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startEncoderRPM */
void startEncoderRPM(void const * argument)
{
  /* USER CODE BEGIN startEncoderRPM */
	uint32_t PreviousWakeTime = osKernelSysTick();

  /* Infinite loop */
	for(;;)
	{
		UpdateEncoderParams(&motor_encoder, TIM3->CNT, RPM_CALCULATE_TIMESTEP_MS);
		osDelayUntil(&PreviousWakeTime, RPM_CALCULATE_TIMESTEP_MS);
	}
  /* USER CODE END startEncoderRPM */
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
	enable_routine = 0;
	silence_alarms = 0;
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
	alarm_flag = 1;
  /* Infinite loop */
	for(;;)
	{
		if (silence_alarms == 1)
		{
			alarm_flag = 0;
			osDelayUntil(&PreviousWakeTime, ALARM_SILENCE_TIMEOUT_MS);
			alarm_flag = 1;
			silence_alarms = 0;
		}
		osDelayUntil(&PreviousWakeTime, ALARM_UPDATE_TIMESTEP_MS);
	}
  /* USER CODE END startAlarmsTask */
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
