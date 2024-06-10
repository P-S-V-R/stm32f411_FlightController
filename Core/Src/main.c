/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************cccc************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
//#include "bmp280.h"
//#include "HMC5883L.h"
#include "stdbool.h"
#include "math.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */


//BMP280_HandleTypedef bmp280;
/*
float pressure, temperature, humidity, altitude_sea, altitude, altitude_cal;
int32_t pressure_rotating_mem[50], pressure_total_avarage;
uint8_t pressure_rotating_mem_location;
float pressure_rotating_mem_actual;
float actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;
*/

volatile long ch[8];
volatile long tick;
volatile uint8_t pulse;

int receiver_input_channel_1;
int receiver_input_channel_2;
int receiver_input_channel_3;
int receiver_input_channel_4;
int receiver_input_channel_5;
int receiver_input_channel_6;

int throttle;

float pid_roll_setpoint;
float pid_pitch_setpoint;
float pid_yaw_setpoint;


float pid_p_gain_roll		= 0.4;			//0.4
float pid_i_gain_roll		= 0.0015;			//0.0015
float pid_d_gain_roll		= 0.7;			//0.7

float pid_p_gain_pitch		= 0.4;			//0.4
float pid_i_gain_pitch		= 0.0015;			//0.0015
float pid_d_gain_pitch		= 0.7;			//0.7

float pid_p_gain_yaw		= 2;
float pid_i_gain_yaw		= 0.02;
float pid_d_gain_yaw		= 0;

float pid_i_mem_roll;
float pid_i_mem_pitch;
float pid_i_mem_yaw;

float pid_last_roll_d_error;
float pid_last_pitch_d_eroor;
float pid_last_yaw_d_error;

float pid_error_temp;

float pid_roll_output;
float pid_pitch_output;
float pid_yaw_output;

int pid_max_roll			= 400;
int pid_max_pitch			= 400;
int pid_max_yaw				= 400;

int esc_1 = 1000;
int esc_2= 1000;
int esc_3= 1000;
int esc_4= 1000;

int arm;

int min_throthle = 1070;
int max_throthle = 2000;
int disable_motor = 1000;

int loop_time;


float turning_speed = 3.0;
bool auto_level = true;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM5_Init();
  MX_TIM11_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);
  HAL_TIM_Base_Start(&htim11);
  HAL_TIM_Base_Start(&htim10);


  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
  HAL_Delay(100);

  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,0);
  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,0);
  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,0);
  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,0);

  HAL_Delay(100);
/*
  HMC5883L_setRange (HMC5883L_RANGE_1_3GA);
  HMC5883L_setMeasurementMode (HMC5883L_CONTINOUS);
  HMC5883L_setDataRate (HMC5883L_DATARATE_15HZ);
  HMC5883L_setSamples (HMC5883L_SAMPLES_1);
  HMC5883L_setOffset (0, 0);

  bmp280_init_default_params(&bmp280.params);
  bmp280.addr = BMP280_I2C_ADDRESS_0;
  bmp280.i2c = &hi2c1;
  bmp280_init(&bmp280, &bmp280.params);
*/
  MPU6050_Init();
  HAL_Delay(1000);
  mpu6050_cal();
  HAL_Delay(1000);
  loop_time = __HAL_TIM_GET_COUNTER(&htim10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //__HAL_TIM_SET_COUNTER(&htim10,0);

	  receiver_input_channel_1 = ch[0];
	  receiver_input_channel_2 = ch[1];
	  receiver_input_channel_3 = ch[2];
	  receiver_input_channel_4 = ch[3];
	  receiver_input_channel_5 = ch[4];
	  receiver_input_channel_6 = ch[5];

	  get_angles();

	  if ( receiver_input_channel_6 > 1550 ){
		  arm = 2;
		  HAL_GPIO_WritePin(led_arm_GPIO_Port, led_arm_Pin, 1);
		  HAL_GPIO_WritePin(led_disarm_GPIO_Port, led_disarm_Pin, 0);

		  pid_i_mem_roll = 0;
		  pid_last_roll_d_error = 0;
		  pid_i_mem_pitch = 0;
		  pid_last_pitch_d_eroor = 0;
		  pid_i_mem_yaw = 0;
		  pid_last_yaw_d_error = 0;
	  }

	  if ( arm == 2 && receiver_input_channel_6 < 1450){
		  arm =0;
		  HAL_GPIO_WritePin(led_arm_GPIO_Port, led_arm_Pin, 0);
		  HAL_GPIO_WritePin(led_disarm_GPIO_Port, led_disarm_Pin, 1);
	  }


	  pid_roll_setpoint =0;
	  if ( receiver_input_channel_1 > 1508 ) {
		  pid_roll_setpoint = (receiver_input_channel_1 - 1508);
	  }
	  else if ( receiver_input_channel_1  < 1492 ){
		  pid_roll_setpoint = ( receiver_input_channel_1  - 1492 );
	  }
	  else{
		  receiver_input_channel_1 = 1500;
	  }

	  pid_roll_setpoint -= roll_level_adjust;
	  pid_roll_setpoint /= turning_speed;

	  pid_pitch_setpoint =0;
	  if ( receiver_input_channel_2 > 1508 ) {
		  pid_pitch_setpoint = ( receiver_input_channel_2 - 1508 );
	  }
	  else if ( receiver_input_channel_2 < 1492 ) {
		  pid_pitch_setpoint = ( receiver_input_channel_2 - 1492 );
	  }
	  else{
	  	receiver_input_channel_2 = 1500;
	  }

	  pid_pitch_setpoint -= pitch_level_adjust;
	  pid_pitch_setpoint /= turning_speed;


	  pid_yaw_setpoint =0;
	  if ( receiver_input_channel_3 > 1050 ){
		  if ( receiver_input_channel_4 > 1508 ) {
			  pid_yaw_setpoint = ( receiver_input_channel_4 - 1508 ) / turning_speed;
		  }
		  else if ( receiver_input_channel_4 < 1492 ){
			  pid_yaw_setpoint = ( receiver_input_channel_4 - 1492 ) / turning_speed;
		  }
	  }
	  else{
	  	  receiver_input_channel_4 = 1500;
	  }

	  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
	  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;

	  if ( pid_i_mem_roll > pid_max_roll ) pid_i_mem_roll = pid_max_roll;
	  else if ( pid_i_mem_roll < pid_max_roll * -1 ) pid_i_mem_roll = pid_max_roll * -1;

	  pid_roll_output = ( pid_p_gain_roll * pid_error_temp ) + pid_i_mem_roll + ( pid_d_gain_roll * ( pid_error_temp - pid_last_roll_d_error));

	  if ( pid_roll_output > pid_max_roll ) pid_roll_output = pid_max_roll;
	  else if ( pid_roll_output < pid_max_roll * -1) pid_roll_output = pid_max_roll * -1;

	  pid_last_roll_d_error = pid_error_temp;


	  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
	  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;

	  if ( pid_i_mem_pitch > pid_max_pitch ) pid_i_mem_pitch = pid_max_pitch;
	  else if ( pid_i_mem_pitch < pid_max_pitch * -1 ) pid_i_mem_pitch = pid_max_pitch * -1;

	  pid_pitch_output = ( pid_p_gain_pitch * pid_error_temp ) + pid_i_mem_pitch + ( pid_d_gain_pitch * ( pid_error_temp - pid_last_pitch_d_eroor));

	  if ( pid_pitch_output > pid_max_pitch ) pid_pitch_output = pid_max_pitch;
	  else if ( pid_pitch_output < pid_max_pitch * -1 ) pid_pitch_output = pid_max_pitch * -1;

	  pid_last_pitch_d_eroor = pid_error_temp;


	  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
	  pid_i_mem_yaw += pid_p_gain_yaw * pid_error_temp;

	  if ( pid_i_mem_yaw > pid_max_yaw ) pid_i_mem_yaw = pid_max_yaw;
	  else if ( pid_i_mem_yaw < pid_max_yaw * -1 ) pid_i_mem_yaw = pid_max_yaw * -1;

	  pid_yaw_output = ( pid_p_gain_yaw * pid_error_temp ) + pid_i_mem_yaw + ( pid_d_gain_yaw * ( pid_error_temp - pid_last_yaw_d_error ));

	  if ( pid_yaw_output > pid_max_yaw ) pid_yaw_output = pid_max_yaw;
	  else if ( pid_yaw_output < pid_max_yaw * -1 ) pid_yaw_output = pid_max_yaw * -1;

	  pid_last_yaw_d_error = pid_error_temp;

	  throttle = receiver_input_channel_3;


	  if ( arm == 2 ){
		  if ( throttle > 1800 ) throttle = 1800;

		  esc_1 = throttle - pid_pitch_output + pid_roll_output - pid_yaw_output;
		  esc_2 = throttle + pid_pitch_output + pid_roll_output + pid_yaw_output;
		  esc_3 = throttle + pid_pitch_output - pid_roll_output - pid_yaw_output;
		  esc_4 = throttle - pid_pitch_output - pid_roll_output + pid_yaw_output;

		  if ( esc_1 < min_throthle ) esc_1 = min_throthle;
		  if ( esc_2 < min_throthle ) esc_2 = min_throthle;
		  if ( esc_3 < min_throthle ) esc_3 = min_throthle;
		  if ( esc_4 < min_throthle ) esc_4 = min_throthle;

		  if ( esc_1 > max_throthle ) esc_1 = max_throthle;
		  if ( esc_2 > max_throthle ) esc_2 = max_throthle;
		  if ( esc_3 > max_throthle ) esc_3 = max_throthle;
		  if ( esc_4 > max_throthle ) esc_4 = max_throthle;


	  }
	  else{
		  esc_1 = disable_motor;
		  esc_2 = disable_motor;
		  esc_3 = disable_motor;
		  esc_4 = disable_motor;
	  }

	  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,esc_1);
	  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,esc_2);
	  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,esc_3);
	  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,esc_4);

	  if(__HAL_TIM_GET_COUNTER(&htim10) - loop_time > 550){
		  //Turn on the LED if the loop time exceeds 550us.
		  HAL_GPIO_WritePin(led_arm_GPIO_Port, led_cal_Pin, 0);
	  }

	  //All the information for controlling the motor's is available.
	  //The refresh rate is 2000Hz. That means the esc's need there pulse every 0.5ms.
	  while(__HAL_TIM_GET_COUNTER(&htim10) - loop_time < 500);                                      //We wait until 500us are passed.
	  loop_time = __HAL_TIM_GET_COUNTER(&htim10);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 100-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 2000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 100-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 0xffff;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 100-1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 0xffff;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */


  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led_cal_GPIO_Port, led_cal_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, led_arm_Pin|led_disarm_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : led_cal_Pin */
  GPIO_InitStruct.Pin = led_cal_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led_cal_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RC_Pin */
  GPIO_InitStruct.Pin = RC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : led_arm_Pin led_disarm_Pin */
  GPIO_InitStruct.Pin = led_arm_Pin|led_disarm_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if ( GPIO_Pin == RC_Pin){
		tick = __HAL_TIM_GET_COUNTER(&htim11);
		__HAL_TIM_SET_COUNTER(&htim11,0);

		if ( tick < 2100){
			ch[pulse] = tick;
			pulse++;
		}
		else{
			__HAL_TIM_SET_COUNTER(&htim11,0);
			pulse =0;
		}

	}

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
