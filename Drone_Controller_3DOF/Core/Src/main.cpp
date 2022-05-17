/* USER CODE BEGIN Header */
/**
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
extern "C" {
	#include "bmp180.h"
}

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



#define I2C_READ 0x01

#ifndef PWM_UPPER
	#define PWM_UPPER 1800
	#define PWM_LOWER 1050
#endif

#define CH_NUM 8
#define CH0 5000
#define EMERGENCY_CH 5

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
float gyroX, gyroY, gyroZ, gyro_e_x, gyroX_a,gyroX_a_x, accX, accY, accZ;
float GyroXh, GyroYh, GyroZh;
float pitch_acc;
float gyroa_x, gyroa_y, gyroa_z;
float alpha, bias;
float alpha_des;
float e, e_eski; //PID hatalari
float ie_roll_sat;

struct state state_des;
struct state state;
struct telem_pack telem_pack;

const float st = 0.005;
//PD Katsayilari

int timer;
int IC_val1, IC_val2, pwm_input;
uint16_t pwm1, pwm2;
uint16_t pwm_mid = 1200;
char buf[sizeof(telem_pack)];
unsigned int micros;
Kalman_Filtresi EKF;
PID pid;
Controller controller;
int controller_output[4];
std::vector<double> controller_output_ang;
unsigned short w1,w2,w3,w4;

bool Is_First_Captured;
volatile int IC_Val2, IC_Val1, Diff, Diff_debug;
volatile short int i;
int ch[CH_NUM+1];
unsigned short int sync;
long int delay_timer, current_time, arm_timer, test_timer, disarm_timer, sent_time;
bool delay_start, arm_start, armed, motor_start, disarm_start;
double w_ang;
float alt;

bmp_t bmp;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void MPU6050_Baslat(void);
int16_t GyroOku (uint8_t addr);
float GyroErr(uint8_t addr);
void TelemPack(void);
void PWMYaz();
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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  MPU6050_Baslat();
  bmp_init(&bmp);
  //Gyro kalibrasyon hatalarını hesapla.
  HAL_Delay(2000);
  GyroXh=GyroErr(GYRO_X_ADDR)/65.5; GyroYh=GyroErr(GYRO_Y_ADDR)/65.5; GyroZh=GyroErr(GYRO_Z_ADDR)/65.5;
  //Kontrolcü Timer'ı
  HAL_TIM_Base_Start_IT(&htim2);

  //Micros timer
  HAL_TIM_Base_Start(&htim3);


  //PWM çıkış timer'ları
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  //PPM Input Capture Kanalları
  //HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
  //printf("Starting...\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //micros = __HAL_TIM_GET_COUNTER(&htim3);
	  //sprintf(buf,"%d\r\n",int(roll)); // @suppress("Float formatting support")
	  if(HAL_GetTick()- sent_time > 1) {
		  TelemPack();
		  HAL_UART_Transmit(&huart2, (uint8_t*)buf, sizeof(struct telem_pack), 1000);
		  char end_char = '@';
		  HAL_UART_Transmit(&huart2, (uint8_t*)&end_char, sizeof(end_char), 1000);
		  HAL_UART_Transmit(&huart2, (uint8_t*)&end_char, sizeof(end_char), 1000);
		  sent_time = HAL_GetTick();

	  }
	  //sprintf(buf,"%s\n","test");

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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  htim1.Init.Prescaler = 32-1;
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
  htim2.Init.Prescaler = 32-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 5000;
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
  htim3.Init.Prescaler = 32-1;
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
  htim4.Init.Period = 65535;
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
	config = 0x04; //0x04
	HAL_I2C_Mem_Write(&hi2c1, (uint16_t)MPU6050, MPU6050_DLPF_REG, 1, &config, 1, 5); //Low Pass Filter 94 Hz'e ayarlandı


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
					armed = true;
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

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

	  telem_pack.attitude_des.roll  = state_des.angles[0];
	  telem_pack.attitude_des.pitch = state_des.angles[1];
	  telem_pack.attitude_des.yaw   = state_des.angles[2];

	  telem_pack.attitude_rate.roll =  state.rates[0];
	  telem_pack.attitude_rate.pitch = state.rates[1];

	  telem_pack.attitude_rate_des.roll =  state_des.rates[0];
	  telem_pack.attitude_rate_des.pitch = state_des.rates[1];

	  telem_pack.ekf.roll_acc  = EKF.roll_acc;
	  telem_pack.ekf.pitch_acc = EKF.pitch_acc;
	  telem_pack.ekf.yaw_acc   = EKF.yaw_acc;

	  telem_pack.ekf.roll_gyro =  EKF.roll_comp;
	  telem_pack.ekf.pitch_gyro = EKF.pitch_comp;

	  telem_pack.pid_roll.P = controller.pid_roll.P;
	  telem_pack.pid_roll.I = controller.pid_roll.I;
	  telem_pack.pid_roll.D = controller.pid_roll.D;
	  telem_pack.pid_roll.pd_roll_sat_buf = controller.pid_roll.pd_roll_sat_buf;

	  telem_pack.pid_pitch.P = controller.pid_pitch.P;
	  telem_pack.pid_pitch.I = controller.pid_pitch.I;
	  telem_pack.pid_pitch.D = controller.pid_pitch.D;
	  telem_pack.pid_pitch.pd_roll_sat_buf = controller.pid_pitch.pd_roll_sat_buf;
	  telem_pack.time_millis = HAL_GetTick();
	  memcpy(buf,&telem_pack,sizeof(telem_pack));
}

int16_t GyroOku (uint8_t addr) {
	uint8_t gyro_data[2];
	HAL_I2C_Mem_Read(&hi2c1, (uint16_t)MPU6050 | I2C_READ, addr, 1, gyro_data, 2, 1);
	int16_t gyro = gyro_data[0]<<8 | gyro_data[1];
	return gyro;
}




void PWMYaz() {
	  if(ch[EMERGENCY_CH-1] < 1500) {
		  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,controller_output[0]);
		  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,controller_output[1]);
		  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,controller_output[2]);
		  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,controller_output[3]);
	  }

	  else {
		  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1000);
		  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1000);
		  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,1000);
		  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,1000);
	  }
}


float GyroErr(uint8_t addr) {
	float GyroXh=0;
	for (int i=0; i<2000; i++)
	{
		GyroXh = (GyroOku(GYRO_X_ADDR));

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


		  gyroX = (GyroOku(GYRO_X_ADDR))/65.5 - GyroXh;
		  gyroY = (GyroOku(GYRO_Y_ADDR))/65.5 - GyroYh;
		  gyroZ = (GyroOku(GYRO_Z_ADDR))/65.5 - GyroZh;
		  //gyroX_a_x = (GyroOku(GYRO_X_ADDR)-gyro_e_x)/65.5;
		  //gyroX_a += gyroX_a_x * st;

		  float gyro[3];
		  gyro[0] = gyroX;
		  gyro[1] = -1*gyroY;
		  gyro[2] = gyroZ;

		  //İvmeölçer degerlerini oku
		  accX = GyroOku(ACC_X_ADDR);
		  accY = GyroOku(ACC_Y_ADDR);
		  accZ = GyroOku(ACC_Z_ADDR);

		  float acc[3];
		  acc[0] = accX;
		  acc[1] = accY;
		  acc[2] = accZ;

		  float acctop=sqrt(accX*accX+accY*accY+accZ*accZ);		//Toplam ivme
		  pitch_acc=asin(accY/acctop)*57.324;					//İvme ölçerden hesaplanan pitch açısı

		  EKF.Run(gyro,acc);
		  state.angles[0]  	  = EKF.state.angles[0];
		  state.angles[1] 	  = EKF.state.angles[1];
		  state.angles[2]     = EKF.state.angles[2];

		  state.rates[0] = gyroX;
		  state.rates[1] = -1*gyroY;
		  state.rates[2] = gyroZ;

		  bmp.uncomp.temp = get_ut ();
		  bmp.data.temp = get_temp (&bmp);
		  bmp.uncomp.press = get_up (bmp.oss);
		  bmp.data.press = get_pressure (bmp);
		  bmp.data.altitude = get_altitude (&bmp);

		  alt = bmp.data.altitude;
		 // alpha_des = 0;
		 // printf("roll: %d\r\n",int(roll));


		  controller_output_ang = controller.Run(state, state_des, ch[2]);
		  controller_output[0] = controller.controller_output_pwm[0];
		  controller_output[1] = controller.controller_output_pwm[1];
		  controller_output[2] = controller.controller_output_pwm[2];
		  controller_output[3] = controller.controller_output_pwm[3];

		  state_des.rates[0] = controller.roll_rate_des;
		  state_des.rates[1] = controller.pitch_rate_des;

		  ie_roll_sat = controller.pid_roll.ie_roll_sat;

		  w_ang = controller.pd_roll;


		  w1 = controller_output[0];
		  w2 = controller_output[1];
		  w3 = controller_output[2];
		  w4 = controller_output[3];

		  if(armed) {
			  if(!motor_start) {
				  MotorBaslat();
				  motor_start = true;
			  }

			  PWMYaz();
		  }

		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);

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
