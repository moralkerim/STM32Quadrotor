/* USER CODE BEGIN Header */
/**
 * TO DO:
 * -Make Qa=0 when disarmed.
 * -Remove GyroXh from equations.
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <main.hpp>
#include "stdio.h"
#include <string.h>
#include <vector>
#include <math.h>

#include "Kalman.hpp"
#include "Controller.hpp"
#include "TelemData.h"
#include "MedianFilter.h"

extern "C" {
	#include "bmp180.h"
	#include "gy-us42v2.h"
}

#include "Modes.h"
#include "state.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MPU6050 (0x68<<1)
#define MPU6050_POW_REG 0x6b
#define MPU6050_DLPF_REG 0x1a
#define GYRO_CONF_REG 0x1b
#define ACC_CONF_REG 0x1c
#define GYRO_X_ADDR 0x43
#define GYRO_Y_ADDR 0x45
#define GYRO_Z_ADDR 0x47
#define ACC_X_ADDR 0x3B
#define ACC_Y_ADDR 0x3D
#define ACC_Z_ADDR 0x3F

#define FULL_CLOCK 800
#define CNTRL_CLOCK 400
#define SONAR_CLOCK 16

#define SONAR_CLOCK_RATE 80


#define I2C_READ 0x01

#ifndef PWM_UPPER
	#define PWM_UPPER 1800
	#define PWM_LOWER 1050
#endif

#define CH_NUM 8
#define CH0 5000
#define EMERGENCY_CH 5
#define MOD_CH 6

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
float gyroX, gyroY, gyroZ, accX, accY, accZ;
float accXc, accYc, accZc;
float accXs, accYs, accZs;
float GyroXh, GyroYh, GyroZh, AccXh, AccYh, AccZh;
float pitch_acc;
float alpha, bias;
float alpha_des;

struct state state_des;
struct state state;
struct telem_pack telem_pack;

const float st = 0.0025;
const float deg2rad = 0.0174;
//PD Katsayilari

int timer;
int IC_val1, IC_val2, pwm_input;
char buf[sizeof(telem_pack)];
unsigned int micros;
Kalman_Filtresi EKF;
PID pid;
Controller controller;
int controller_output[4];
std::vector<float> controller_output_ang;
unsigned short w1,w2,w3,w4;

bool Is_First_Captured;
volatile int IC_Val2, IC_Val1, Diff, Diff_debug;
volatile short int i;
int ch[CH_NUM+1];
unsigned short int sync;
long int delay_timer, current_time, arm_timer, test_timer, disarm_timer, sent_time;
bool delay_start, arm_start, armed, motor_start, disarm_start, sonar_ready;
double w_ang;
float baro_alt, sonar_alt, sonar_alt_, sonar_vel, sonar_vel_, sonar_acc, alt, alt_gnd, vz, baro_gnd;
unsigned int sonar_range;
unsigned short int controller_counter, sonar_counter, camera_counter;
bmp_t bmp;
float z0;

struct cam_data cam_data;
struct cam_data cam_data_20;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void MPU6050_Baslat(void);
int16_t GyroOku (uint8_t addr);
float GyroErr(uint8_t addr);
void TelemPack(void);
void SendTelem(void);
void PWMYaz();
void checkMode(int mod_ch);
void MotorBaslat(void);
void Check_Arm(void);
void Check_Disarm(void);
int _write(int32_t file, uint8_t *ptr, int32_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int __io_putchar(int ch)
{
 // Write character to ITM ch.0
 ITM_SendChar(ch);
 return(ch);
}

void Delay(long millis) {
	current_time = delay_timer;
	while(delay_timer < current_time + millis) {
		printf("Do nothing...");
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
	//Delay(500);
	//HAL_Delay(1000);

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  HAL_Delay(2000);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_DMA(&huart1, (uint8_t*)&cam_data, sizeof(cam_data));
  //HAL_UART_Receive_DMA(&huart1, rx_buffer, sizeof(cam_data)-1);
  MPU6050_Baslat();
  bmp_init(&bmp);
  //Gyro kalibrasyon hatalarını hesapla.
  HAL_Delay(2000);
  //EKF.roll_bias=GyroErr(GYRO_X_ADDR)/65.5; EKF.pitch_bias=-1*GyroErr(GYRO_Y_ADDR)/65.5;
  GyroXh = GyroErr(GYRO_X_ADDR); GyroYh=GyroErr(GYRO_Y_ADDR); GyroZh=GyroErr(GYRO_Z_ADDR);
  AccXh = GyroErr(ACC_X_ADDR)/4096; AccYh = GyroErr(ACC_Y_ADDR)/4096; AccZh = GyroErr(ACC_Z_ADDR)/4096;

  AccXh = 0.815*AccXh - 0.42592*AccYh - 0.072464*AccZh + 0.001334;
  AccYh = 0.96009*AccYh - 0.42592*AccXh + 0.0091315*AccZh + 0.042165;
  AccZh = 0.0091315*AccYh - 0.072464*AccXh + 0.98549*AccZh + 0.08443;

  //İvmeölçer degerlerini oku
  accX = GyroOku(ACC_X_ADDR);
  accY = GyroOku(ACC_Y_ADDR);
  accZ = GyroOku(ACC_Z_ADDR);

  /*
  float acctop=sqrt(accX*accX+accY*accY+accZ*accZ);

  float rad2deg = 57.3248;
  EKF.PITCH_OFFSET = -1 * asin(accX/acctop)*rad2deg;
  EKF.ROLL_OFFSET  = -1 * asin(accY/acctop)*rad2deg;
  */

  //Kontrolcü Timer'ı
  HAL_TIM_Base_Start_IT(&htim2);

  //Micros timer
  HAL_TIM_Base_Start(&htim3);


  //PWM çıkış timer'ları
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  //PPM Input Capture Kanalları
  //HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
  //printf("Starting...\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  SendTelem();
	  Check_Arm();
	  Check_Disarm();


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1250;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 32000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 1000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void MPU6050_Baslat(void) {
	uint8_t config = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, (uint16_t)MPU6050, MPU6050_POW_REG, 1, &config, 1, 5); //Güç registerını aktif et
	config = 0x08;
	HAL_I2C_Mem_Write(&hi2c1, (uint16_t)MPU6050, GYRO_CONF_REG, 1, &config, 1, 5); //Gyro 250 d/s'ye ayarlandi.
	config = 0x10;
	HAL_I2C_Mem_Write(&hi2c1, (uint16_t)MPU6050, ACC_CONF_REG, 1, &config, 1, 5); //Acc +-8g'ye ayarlandi.
	//config = 0x04; //0x04
	//HAL_I2C_Mem_Write(&hi2c1, (uint16_t)MPU6050, MPU6050_DLPF_REG, 1, &config, 1, 5); //Low Pass Filter 94 Hz'e ayarlandı


}

void checkMode(int mod_ch) {
	  if(mod_ch < 1400) {

		  controller.mod = STABILIZE;
		  controller.z0 = EKF.alt_gnd;
		  controller.p_alt.reset();
	  }

	  else if (mod_ch >=1400 && mod_ch <1700) {
		  //Run (struct state state, struct state state_des, float z_vel, float z0, float z, float ch3),
		  controller.mod = ALT_HOLD;

		  //z0 = controller.p_alt.zi;

	  }

	  else {
		  controller.mod = ALT_HOLD;
	  }
}

void Check_Arm() {
	if(!armed) {
		if((ch[2] < 1100) && (ch[3] > 1700)) {
				if(!arm_start){
					arm_timer = HAL_GetTick();
					arm_start = true;
				}

				if(HAL_GetTick() - arm_timer > 3000) {
					controller.pid_roll.reset();
					controller.pid_pitch.reset();
					controller.pid_yaw.reset();
					armed = true;
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
					EKF.sb = 1e-3;
					EKF.Qa = 5e4;

					/*
					controller.pid_roll.angle0   = EKF.state.angles[0];
					controller.pid_pitch.angle0  = EKF.state.angles[1];
					*/

				}

		}

		else {
			arm_start = false;
		}
	}

}

void Check_Disarm() {
	if(armed) {
		if((ch[2] < 1100) && (ch[3] < 1100)) {
				if(!disarm_start){
					disarm_timer = HAL_GetTick();
					disarm_start = true;
				}

				if(HAL_GetTick() - disarm_timer > 3000) {
					armed = false;
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

				}

		}

		else {
			disarm_start = false;
		}
	}
}

void TelemPack() {
	  telem_pack.attitude.roll  = state.angles[0];
	  telem_pack.attitude.pitch = state.angles[1];
	  telem_pack.attitude.yaw   = state.angles[2];

	  telem_pack.pwm.w1 = controller_output[0];
	  telem_pack.pwm.w2 = controller_output[1];
	  telem_pack.pwm.w3 = controller_output[2];
	  telem_pack.pwm.w4 = controller_output[3];

	  telem_pack.attitude_des.roll  = controller.roll_des;
	  telem_pack.attitude_des.pitch = controller.pitch_des;
	  telem_pack.attitude_des.yaw   = 0;

	  telem_pack.attitude_rate.roll  = state.rates[0];
	  telem_pack.attitude_rate.pitch = state.rates[1];
	  telem_pack.attitude_rate.yaw 	 = state.rates[2];

	  telem_pack.attitude_rate_des.roll =  state_des.rates[0];
	  telem_pack.attitude_rate_des.pitch = state_des.rates[1];

	  telem_pack.ekf.roll_acc  = EKF.roll_acc;
	  telem_pack.ekf.pitch_acc = EKF.pitch_acc;

	  telem_pack.ekf.roll_gyro  = gyroX;
	  telem_pack.ekf.pitch_gyro = gyroY;

	  telem_pack.ekf.roll_comp =  EKF.roll_comp;
	  telem_pack.ekf.pitch_comp = EKF.pitch_comp;

	  telem_pack.ekf.roll_ekf =  EKF.roll_ekf;
	  telem_pack.ekf.pitch_ekf = EKF.pitch_ekf;

	  telem_pack.pid_roll.P = controller.pid_roll.P;
	  telem_pack.pid_roll.I = controller.pid_roll.I;
	  telem_pack.pid_roll.D = controller.pid_roll.D;
	  telem_pack.pid_roll.pd_roll_sat_buf = controller.pid_roll.pd_roll_sat_buf;

	  telem_pack.pid_pitch.P = controller.pid_pitch.P;
	  telem_pack.pid_pitch.I = controller.pid_pitch.I;
	  telem_pack.pid_pitch.D = controller.pid_pitch.D;
	  telem_pack.pid_pitch.pd_roll_sat_buf = controller.pid_pitch.pd_roll_sat_buf;

	  telem_pack.sonar_alt = EKF.sonar_alt;
	  telem_pack.velocity_body.z = EKF.vz;
	  telem_pack.position_body.z = EKF.alt_gnd;

	  telem_pack.cam_data.detected = cam_data_20.detected;
	  telem_pack.cam_data.x = cam_data_20.x;
	  telem_pack.cam_data.y = cam_data_20.y;
	  telem_pack.cam_data.z_cam = cam_data_20.z_cam;


	  telem_pack.position_body.x = EKF.xpos;
	  telem_pack.velocity_body.x = EKF.vx;
	  //telem_pack.position_body.y = EKF.ypos;

	  telem_pack.alt_thr = controller.alt_thr;

	  telem_pack.time_millis = HAL_GetTick();

	  telem_pack.acc.x = accXc;
	  telem_pack.acc.y = accYc;
	  telem_pack.acc.z = accZc;
	  memcpy(buf,&telem_pack,sizeof(telem_pack));
}

void SendTelem() {
	  TelemPack();
	  HAL_UART_Transmit(&huart2, (uint8_t*)buf, sizeof(struct telem_pack), 100);
	  char end_char = '@';
	  HAL_UART_Transmit(&huart2, (uint8_t*)&end_char, sizeof(end_char), 100);
	  HAL_UART_Transmit(&huart2, (uint8_t*)&end_char, sizeof(end_char), 100);
	  sent_time = HAL_GetTick();


}

int16_t GyroOku (uint8_t addr) {
	uint8_t gyro_data[2];
	HAL_I2C_Mem_Read(&hi2c1, (uint16_t)MPU6050 | I2C_READ, addr, 1, gyro_data, 2, 1);
	int16_t gyro = gyro_data[0]<<8 | gyro_data[1];
	return gyro;
}




void PWMYaz() {
	  if(armed) {

		  if(!motor_start) {
		  	  MotorBaslat();
		  	  motor_start = true;
		  }

		  if(ch[EMERGENCY_CH-1] < 1500 && ch[3-1] > 1050) {


			  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,controller_output[0]);
			  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,controller_output[1]);
			  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,controller_output[2]);
			  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,controller_output[3]);
		  }

		  else if(motor_start) {
			  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1000);
			  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1000);
			  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,1000);
			  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,1000);

		  }
	  }



}


float GyroErr(uint8_t addr) {
	float GyroXh=0;
	for (int i=0; i<2000; i++)
	{
		GyroXh += (GyroOku(addr));

	} //Haberleşmeyi durdur.
	GyroXh=GyroXh/2000; //Son okunan değeri 2000'e böl.
	return GyroXh;
}

void MotorBaslat(void) {
	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1000);
	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1000);
	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,1000);
	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,1000);
	 // Delay(600);
	HAL_Delay(1000);
}

int _write(int32_t file, uint8_t *ptr, int32_t len) {
	/* Implement your write code here, this is used by puts and printf for example */
	int i = 0;
	for(i=0; i < len; i++) {
		ITM_SendChar((*ptr++));
	}
	return len;
}

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim) {

	if(htim == &htim2) {
		//1.25 ms || 800 Hz


		set_ucounter(SONAR_CLOCK_RATE);
		set_b_counter(12);

		controller_counter++;
		camera_counter++;

		if(camera_counter == 40) {
			  camera_counter = 0;
			  memcpy(&cam_data_20, &cam_data, sizeof(cam_data));
			  EKF.camx = (float)cam_data.y/100.0;

			  if(!cam_data.detected) {
				  EKF.Qc = 9e9;
			  }

			  else {
				  EKF.Qc = 2.7e-2;
			  }
		}

		if(get_ucounter() == 1) {
			request_range();
			//sonar_range = getRange();
		}


		else if (get_ucounter() == SONAR_CLOCK_RATE) {
		  sonar_range = getRange();
		  sonar_alt_ = sonar_alt;
		  sonar_vel_ = sonar_vel;

		  float sonar_roll = abs(deg2rad*state.angles[0]);
		  float sonar_pitch = abs(deg2rad*state.angles[1]);
		  sonar_alt = (float)sonar_range/100.0 * cos(sonar_roll)* cos(sonar_pitch);
		  float sonar_st = (float)(1.0/SONAR_CLOCK);
		  sonar_vel = (sonar_alt - sonar_alt_)/sonar_st;


		  if (abs(sonar_vel) > 7) {
			  sonar_alt = sonar_alt_;
			  sonar_vel = sonar_vel_;
		  }

		  if(sonar_alt > 6 || sonar_alt < 0.3) {
			  EKF.Qs = 9e9;
			  EKF.salt = 50;
		  }

		  else {
			  EKF.Qs = 0.25;
			  EKF.salt = 1;
		  }


		}

		if(get_b_counter() == 1) {
			write_ut();
		}

		else if(get_b_counter() == 5) { //5 ms
			bmp.uncomp.temp = read_ut ();
			bmp.data.temp = get_temp (&bmp);
			write_up();
		}

		else if(get_b_counter() == 12) { //
			bmp.uncomp.press = read_up (bmp.oss);
			bmp.data.press = get_pressure (bmp);
			bmp.data.altitude = get_altitude (&bmp);
			baro_alt = bmp.data.altitude;


		}

		//}

		if(controller_counter == 2) { //2.5 ms || 400 Hz

		  controller_counter = 0;



		  gyroX = (GyroOku(GYRO_X_ADDR)- GyroXh)/65.5 ;
		  gyroY = (GyroOku(GYRO_Y_ADDR)- GyroYh)/65.5 ;
		  gyroZ = (GyroOku(GYRO_Z_ADDR)- GyroZh)/65.5 ;
		  //gyroX_a_x = (GyroOku(GYRO_X_ADDR)-gyro_e_x)/65.5;
		  //gyroX_a += gyroX_a_x * st;



		  //float gyro[3];
		  EKF.gyro[0] = gyroX;
		  EKF.gyro[1] = -1*gyroY;
		  EKF.gyro[2] = gyroZ;

		  //İvmeölçer degerlerini oku
		  accX = GyroOku(ACC_X_ADDR);
		  accY = GyroOku(ACC_Y_ADDR);
		  accZ = GyroOku(ACC_Z_ADDR);

		  accXs = (float)accX/4096.0;
		  accYs = (float)accY/4096.0;
		  accZs = (float)accZ/4096.0;

		  accXc = 0.815*accXs - 0.42592*accYs - 0.072464*accZs + 0.001334;
		  accYc = 0.96009*accYs - 0.42592*accXs + 0.0091315*accZs + 0.042165;
		  accZc = 0.0091315*accYs - 0.072464*accXs + 0.98549*accZs + 0.08443;

		  //float acc[3];
		  EKF.acc[0] = accXc;// - AccXh;
		  EKF.acc[1] = accYc;// - AccYh;
		  EKF.acc[2] = accZc;// - AccZh;

		  //float acctop=sqrt(accX*accX+accY*accY+accZ*accZ);		//Toplam ivme
		 // pitch_acc=asin(accY/acctop)*57.324;					//İvme ölçerden hesaplanan pitch açısı

		  float g = 9.81;
		  EKF.acc_vert = (accZc - 0.99)  * g;

		  float ax_b = (accXc-AccXh);
		  ax_b = ax_b - 1 * sin(deg2rad*EKF.state.angles[1]);
		  ax_b = ax_b * cos(deg2rad*EKF.state.angles[1]);
		  float accXm = ax_b  * g;
		  float accYm = (accYc-AccYh)  * g;

		  EKF.accXm = accXm;// * deg2rad*EKF.state.angles[1];
		  EKF.accYm = accYm;

		  EKF.sonar_alt = sonar_alt;
		  EKF.baro_alt = baro_alt;

		  EKF.Run();

/*
		  if(sonar_alt > 5 || sonar_alt < 0.3) {
			  Qs = 1e4;
		  }
*/


		  state.angles[0]  	  = EKF.state.angles[0];
		  state.angles[1] 	  = EKF.state.angles[1];
		  state.angles[2]     = EKF.state.angles[2];

		  state.rates[0] = EKF.state.rates[0];
		  state.rates[1] = EKF.state.rates[1];
		  state.rates[2] = EKF.state.rates[2];
/*
		  bmp.uncomp.temp = get_ut ();
		  bmp.data.temp = get_temp (&bmp);
		  bmp.uncomp.press = get_up (bmp.oss);
		  bmp.data.press = get_pressure (bmp);
		  bmp.data.altitude = get_altitude (&bmp);

		  baro_alt = bmp.data.altitude; */


		 // alpha_des = 0;
		 // printf("roll: %d\r\n",int(roll));

			checkMode(ch[MOD_CH-1]);

			controller.z_vel = EKF.vz;
			controller.vx	 = EKF.vx;
			controller.x     = EKF.xpos;
			//controller.z0 = z0;
			controller.z = EKF.alt_gnd;

		  controller.state = state;
		  controller.state_des = state_des;
		  controller.ch3 = ch[2];

		  controller_output_ang = controller.Run();

		  controller_output[0] = controller.controller_output_pwm[0];
		  controller_output[1] = controller.controller_output_pwm[1];
		  controller_output[2] = controller.controller_output_pwm[2];
		  controller_output[3] = controller.controller_output_pwm[3];

		  state_des.rates[0] = controller.roll_rate_des;
		  state_des.rates[1] = controller.pitch_rate_des;

		  //ie_roll_sat = controller.pid_roll.ie_roll_sat;

		  w_ang = controller.pd_roll;


		  w1 = controller_output[0];
		  w2 = controller_output[1];
		  w3 = controller_output[2];
		  w4 = controller_output[3];


		  PWMYaz();


		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
		}
		}
	}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)

{

 if(htim == &htim3) {


	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)  // if the interrupt source is channel1
	{
				IC_Val1 = IC_Val2;
				IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);  // read second value
				Diff = IC_Val2-IC_Val1;
				if(Diff < 0) {
					Diff+=65535;

				}
				//printf("Diff: %d\n",Diff);
				ch[i] = Diff;
				if(1) {
					if(ch[i] > CH0) {
						//ch[CH_NUM] = ch[i];
						i = -1;
						sync = 1;

					}
				}



				state_des.angles[0] =  pid.pwm2ang(ch[0]);
				state_des.angles[1] =  pid.pwm2ang(ch[1]);
				state_des.angles[2] =  0;
				state_des.rates[2] = pid.pwm2rate(ch[3]);

				i++;
				i = i % (CH_NUM+1);
			//	__HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_2,TIM_INPUTCHANNELPOLARITY_RISING);


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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
