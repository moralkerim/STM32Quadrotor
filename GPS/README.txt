Instructions to implement GPS library:

1. Add the source and header files.
2. Create a variable to store gps data -> GPSSTRUCT gpsData;
3. Initilize ring buffer code ->  Ringbuf_init ();
	Note: After intilizing the ring buffer some delay is necessary for the buffer to fill.
4. Get the gps data and stor them in the vairable -> getGPSData(&gpsData);

IMPORTANT NOTE: MUST ADD THE FOLLOWING LINES IN stm32f1xx_it.c:

/* USER CODE BEGIN 0 */
extern void Uart_isr (UART_HandleTypeDef *huart);
extern volatile uint16_t timeout;
/* USER CODE END 0 */

void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  if(timeout >0)  timeout--;

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
  Uart_isr (&huart3);
  /* USER CODE END USART3_IRQn 0 */
  //HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}