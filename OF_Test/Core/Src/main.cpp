/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct OF_Msg {
  int32_t motion_x;
  int32_t motion_y;
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define START_MSG 0x24
#define RNG_MSG_L 0x01
#define RNG_MSG_U 0x1f
#define OF_MSG_L 0x02
#define OF_MSG_U 0x1f
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t inChar;
uint8_t matek_msg[50];
uint8_t count=0;
uint16_t msg_type;
uint16_t range_msg = RNG_MSG_U << 8 | RNG_MSG_L;
uint16_t of_msg = OF_MSG_U << 8 | OF_MSG_L;
int i = 0;
bool reset;
uint8_t size;
int32_t distance;
uint16_t payload_size;
OF_Msg of_msg_str;
uint8_t quality;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int32_t MatekDecodeRange(uint8_t* msg) {
  int32_t distance = msg[9] | msg[10] << 8 | msg[11] << 16 | msg[12] << 24 ;
  return distance;
}

OF_Msg MatekDecodeOF(uint8_t* msg) {
  OF_Msg of_msg;
  of_msg.motion_x = msg[9] | msg[10] << 8 | msg[11] << 16 | msg[12] << 24;
  of_msg.motion_y = msg[13] | msg[14] << 8 | msg[15] << 16 | msg[16] << 24 ;
  return of_msg;
}

uint16_t MatekDecodeOF2(uint8_t* msg) {
  uint16_t mot_x = msg[9] | msg[10] << 8 | msg[11] << 16 | msg[12] << 24 ;
  return mot_x;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart1.Instance)
	{

		HAL_UART_Receive_DMA (&huart1, (uint8_t*)&inChar, 1);
		count++;
		if(inChar == START_MSG) {
				count = 0;
				matek_msg[count] = START_MSG;
				if((matek_msg[12] | matek_msg[11] | matek_msg[10] | matek_msg[9]) != 0) {
					msg_type = matek_msg[5] << 8 | matek_msg[4];
					quality = matek_msg[8];
					if(msg_type == range_msg) {
						distance = MatekDecodeRange(matek_msg);
						//payload_size = matek_msg[7] << 8 | matek_msg[6];

					}

					else if(msg_type == of_msg) {
						of_msg_str = MatekDecodeOF(matek_msg);
					}
				}
		}
		else {
			matek_msg[count] = inChar;
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_DMA (&huart1, (uint8_t*)&inChar, 1);
	count++;
	if(inChar == START_MSG) {
			count = 0;
			matek_msg[count] = START_MSG;
			if((matek_msg[12] | matek_msg[11] | matek_msg[10] | matek_msg[9]) != 0) {
				msg_type = matek_msg[5] << 8 | matek_msg[4];
				quality = matek_msg[8];
				if(msg_type == range_msg) {
					distance = MatekDecodeRange(matek_msg);
					payload_size = matek_msg[7] << 8 | matek_msg[6];

				}
			}
	}
	else {
		matek_msg[count] = inChar;
	}
  /* USER CODE END 2 */

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

/* USER CODE BEGIN 4 */

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
