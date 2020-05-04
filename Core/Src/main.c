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
#include "spi_pressure_snsr_api.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_RPM_CALCULATE_TIMESTEP_MS			10
#define ARM_RPM_CALCULATE_TIMESTEP_MS			10
#define LCD_DISPLAY_UPDATE_TIMESTEP_MS			100
#define ALARM_UPDATE_TIMESTEP_MS				10
#define ALARM_SILENCE_TIMEOUT_MS				2000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart5;

osThreadId initTaskHandle;
osThreadId mainRoutineHandle;
osThreadId displayUIHandle;
osThreadId motorEncoderHandle;
osThreadId armEncoderHandle;
osThreadId calibRoutineHandle;
osThreadId diagnosticsSrvHandle;
osThreadId userInputHandle;
osThreadId alarmMonitorHandle;
osThreadId pressureSnsrHandle;
/* USER CODE BEGIN PV */

DCMotor_S 			dc_motor;
LCDDisplay_S 		lcd_display;
Encoder_S			motor_encoder;
Buzzer_S			buzzer;
Potentiometer_S		pot_controls_a[TOTAL_CONTROLS_COUNT];
Ventilator_S		ventilator;
SPIPressureSensor_S pressure_sensor;

uint32_t			prev_systick = 0;
volatile uint8_t 	unwind_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM10_Init(void);
static void MX_UART5_Init(void);
void initTaskFn(void const * argument);
void mainRoutineFn(void const * argument);
void displayUIFn(void const * argument);
void motorEncoderFn(void const * argument);
void armEncoderFn(void const * argument);
void calibRoutineFn(void const * argument);
void diagnosticsSrvFn(void const * argument);
void userInputFn(void const * argument);
void alarmMonitorFn(void const * argument);
void pressureSnsrFn(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if (GPIO_Pin == EditBtnIn_Pin)
	{
		osThreadResume(userInputHandle);
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM10_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  DCMotorInit(&dc_motor, &htim5);
  EncoderInit(&motor_encoder, HD_MODEL);
  LCDInit(&lcd_display, &hi2c1);
  PotControlsInit(pot_controls_a);
  BuzzerInit(&buzzer);
  SPIPrsrSnsrInit(&pressure_sensor, &hspi1);
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
  /* definition and creation of initTask */
  osThreadDef(initTask, initTaskFn, osPriorityRealtime, 0, 128);
  initTaskHandle = osThreadCreate(osThread(initTask), NULL);

  /* definition and creation of mainRoutine */
  osThreadDef(mainRoutine, mainRoutineFn, osPriorityHigh, 0, 128);
  mainRoutineHandle = osThreadCreate(osThread(mainRoutine), NULL);

  /* definition and creation of displayUI */
  osThreadDef(displayUI, displayUIFn, osPriorityHigh, 0, 128);
  displayUIHandle = osThreadCreate(osThread(displayUI), NULL);

  /* definition and creation of motorEncoder */
  osThreadDef(motorEncoder, motorEncoderFn, osPriorityHigh, 0, 128);
  motorEncoderHandle = osThreadCreate(osThread(motorEncoder), NULL);

  /* definition and creation of armEncoder */
  osThreadDef(armEncoder, armEncoderFn, osPriorityHigh, 0, 128);
  armEncoderHandle = osThreadCreate(osThread(armEncoder), NULL);

  /* definition and creation of calibRoutine */
  osThreadDef(calibRoutine, calibRoutineFn, osPriorityRealtime, 0, 128);
  calibRoutineHandle = osThreadCreate(osThread(calibRoutine), NULL);

  /* definition and creation of diagnosticsSrv */
  osThreadDef(diagnosticsSrv, diagnosticsSrvFn, osPriorityHigh, 0, 128);
  diagnosticsSrvHandle = osThreadCreate(osThread(diagnosticsSrv), NULL);

  /* definition and creation of userInput */
  osThreadDef(userInput, userInputFn, osPriorityHigh, 0, 128);
  userInputHandle = osThreadCreate(osThread(userInput), NULL);

  /* definition and creation of alarmMonitor */
  osThreadDef(alarmMonitor, alarmMonitorFn, osPriorityRealtime, 0, 128);
  alarmMonitorHandle = osThreadCreate(osThread(alarmMonitor), NULL);

  /* definition and creation of pressureSnsr */
  osThreadDef(pressureSnsr, pressureSnsrFn, osPriorityHigh, 0, 128);
  pressureSnsrHandle = osThreadCreate(osThread(pressureSnsr), NULL);

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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim3.Init.Period = 1600-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 2;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
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
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 2;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
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

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 8000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 0;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */
  HAL_TIM_MspPostInit(&htim10);

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_2;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, AlarmLED3_Pin|AlarmLED4_Pin|AlarmLED5_Pin|AlarmLED6_Pin 
                          |AlarmLED7_Pin|AlarmLED1_Pin|AlarmLED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, AlarmLED8_Pin|SysOnLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, MotorCW_Pin|MotorCCW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, PSnsrCSADCOut_Pin|PSnsrCSEEOut_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : AlarmLED3_Pin AlarmLED4_Pin AlarmLED5_Pin AlarmLED6_Pin 
                           AlarmLED7_Pin AlarmLED1_Pin AlarmLED2_Pin */
  GPIO_InitStruct.Pin = AlarmLED3_Pin|AlarmLED4_Pin|AlarmLED5_Pin|AlarmLED6_Pin 
                          |AlarmLED7_Pin|AlarmLED1_Pin|AlarmLED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AlarmLED8_Pin SysOnLED_Pin */
  GPIO_InitStruct.Pin = AlarmLED8_Pin|SysOnLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PrsrSnsrDRDY_Pin */
  GPIO_InitStruct.Pin = PrsrSnsrDRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(PrsrSnsrDRDY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : StartStopBtnIn_Pin EditBtnIn_Pin CalibrationBtnIn_Pin AlarmSilenceBtnIn_Pin */
  GPIO_InitStruct.Pin = StartStopBtnIn_Pin|EditBtnIn_Pin|CalibrationBtnIn_Pin|AlarmSilenceBtnIn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : MotorCW_Pin MotorCCW_Pin */
  GPIO_InitStruct.Pin = MotorCW_Pin|MotorCCW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PSnsrCSADCOut_Pin PSnsrCSEEOut_Pin */
  GPIO_InitStruct.Pin = PSnsrCSADCOut_Pin|PSnsrCSEEOut_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_initTaskFn */
/**
  * @brief  Function implementing the initTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_initTaskFn */
void initTaskFn(void const * argument)
{
  /* USER CODE BEGIN 5 */
	VentilatorInit(&ventilator);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	osThreadResume(userInputHandle);
	osThreadTerminate(initTaskHandle);
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_mainRoutineFn */
/**
* @brief Function implementing the mainRoutine thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_mainRoutineFn */
void mainRoutineFn(void const * argument)
{
  /* USER CODE BEGIN mainRoutineFn */
	uint32_t PreviousWakeTime = osKernelSysTick();

  /* Infinite loop */
	for(;;)
	{
		if (((ventilator.status_flags & ENABLE_ROUTINE) == ENABLE_ROUTINE) && !((ventilator.status_flags & START_CALIBRATION) == START_CALIBRATION))
		{
			dc_motor.pwm_value = ventilator.motor_pwm_value_in;
			dc_motor.direction_flag = MOTOR_SPIN_CCW;
			DCMotorRPMSet(&dc_motor);

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
			osDelayUntil(&PreviousWakeTime, 10);
		}
	}
  /* USER CODE END mainRoutineFn */
}

/* USER CODE BEGIN Header_displayUIFn */
/**
* @brief Function implementing the displayUI thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_displayUIFn */
void displayUIFn(void const * argument)
{
  /* USER CODE BEGIN displayUIFn */
	char buffer[32];
	uint32_t PreviousWakeTime = osKernelSysTick();
	osThreadResume(userInputHandle);

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
  /* USER CODE END displayUIFn */
}

/* USER CODE BEGIN Header_motorEncoderFn */
/**
* @brief Function implementing the motorEncoder thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motorEncoderFn */
void motorEncoderFn(void const * argument)
{
  /* USER CODE BEGIN motorEncoderFn */
	uint32_t PreviousWakeTime = osKernelSysTick();

  /* Infinite loop */
	for(;;)
	{
		/* TODO: Refactor of encoder parameters update function */
		//UpdateEncoderParams(&motor_encoder, TIM3->CNT, MOTOR_RPM_CALCULATE_TIMESTEP_MS);
		osDelayUntil(&PreviousWakeTime, MOTOR_RPM_CALCULATE_TIMESTEP_MS);
	}
  /* USER CODE END motorEncoderFn */
}

/* USER CODE BEGIN Header_armEncoderFn */
/**
* @brief Function implementing the armEncoder thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_armEncoderFn */
void armEncoderFn(void const * argument)
{
  /* USER CODE BEGIN armEncoderFn */
	uint32_t PreviousWakeTime = osKernelSysTick();

  /* Infinite loop */
	for(;;)
	{
		/* TODO: Refactor of encoder parameters update function */
		//UpdateEncoderParams(&motor_encoder, TIM4->CNT, ARM_RPM_CALCULATE_TIMESTEP_MS);
		osDelayUntil(&PreviousWakeTime, ARM_RPM_CALCULATE_TIMESTEP_MS);
	}
  /* USER CODE END armEncoderFn */
}

/* USER CODE BEGIN Header_calibRoutineFn */
/**
* @brief Function implementing the calibRoutine thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_calibRoutineFn */
void calibRoutineFn(void const * argument)
{
  /* USER CODE BEGIN calibRoutineFn */
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
  /* USER CODE END calibRoutineFn */
}

/* USER CODE BEGIN Header_diagnosticsSrvFn */
/**
* @brief Function implementing the diagnosticsSrv thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_diagnosticsSrvFn */
void diagnosticsSrvFn(void const * argument)
{
  /* USER CODE BEGIN diagnosticsSrvFn */
	osThreadSuspend(diagnosticsSrvHandle);
	char buffer[21];
	uint32_t PreviousWakeTime = osKernelSysTick();

  /* Infinite loop */
	for(;;)
	{
		if ((ventilator.status_flags & ENABLE_ROUTINE) == ENABLE_ROUTINE)
		{
			sprintf(buffer, "%04lu,%04lu,%04lu,%04u", HAL_GetTick(), TIM3->CNT, TIM4->CNT, dc_motor.pwm_value);
			HAL_UART_Transmit(&huart5, (uint8_t *) buffer, sizeof(buffer), 100);
		}
		osDelayUntil(&PreviousWakeTime, 10);
	}
  /* USER CODE END diagnosticsSrvFn */
}

/* USER CODE BEGIN Header_userInputFn */
/**
* @brief Function implementing the userInput thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_userInputFn */
void userInputFn(void const * argument)
{
  /* USER CODE BEGIN userInputFn */
  /* Infinite loop */
	for(;;)
	{
	  uint16_t adc_values[TOTAL_CONTROLS_COUNT] = {0,0,0,0};
	  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_values, TOTAL_CONTROLS_COUNT);
	  PotControlsValueUpdate(pot_controls_a, adc_values);
	  UpdateVentilatorParams(&ventilator, pot_controls_a);
	  osThreadSuspend(userInputHandle);
	}
  /* USER CODE END userInputFn */
}

/* USER CODE BEGIN Header_alarmMonitorFn */
/**
* @brief Function implementing the alarmMonitor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_alarmMonitorFn */
void alarmMonitorFn(void const * argument)
{
  /* USER CODE BEGIN alarmMonitorFn */
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
  /* USER CODE END alarmMonitorFn */
}

/* USER CODE BEGIN Header_pressureSnsrFn */
/**
* @brief Function implementing the pressureSnsr thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_pressureSnsrFn */
void pressureSnsrFn(void const * argument)
{
  /* USER CODE BEGIN pressureSnsrFn */
	HAL_GPIO_WritePin(PSnsrCSADCOut_GPIO_Port, PSnsrCSADCOut_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PSnsrCSEEOut_GPIO_Port, PSnsrCSEEOut_Pin, GPIO_PIN_RESET);

	uint8_t buffer_tx[2];
	uint8_t buffer_rx[13];

	buffer_tx[0] = EAD_EEPROM_CMD;
	buffer_tx[1] = PRESSURE_RANGE_BYTE_2;

	HAL_SPI_TransmitReceive_DMA(pressure_sensor.spi_handle, buffer_tx, buffer_rx, 2);

  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END pressureSnsrFn */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
