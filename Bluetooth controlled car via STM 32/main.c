/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
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
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rx_buff[3];
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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

  int speed=500;

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    /* USER CODE END WHILE */
		  HAL_UART_Receive_IT(&huart1,rx_buff,3);
		  HAL_UART_Transmit(&huart2,rx_buff,3,10);
		  if(rx_buff[0]=='F')
		  {

		    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
		    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
		    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);
		    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1);

		    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
		    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
		    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
		    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);

		  	rx_buff[0]=0;
		  	rx_buff[1]=0;
		  	rx_buff[2]=0;

		  }

		  else if(rx_buff[0]=='B')
		  {
			    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
			    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
			    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1);
			    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);

			    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
			    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
			    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
			    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);

			  	rx_buff[0]=0;
			  	rx_buff[1]=0;
			  	rx_buff[2]=0;

		  }

		  else if(rx_buff[0]=='R')
		  {
			    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
			    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
			    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1);
			    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);

			    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
			    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
			    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
			    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);

			  	rx_buff[0]=0;
			  	rx_buff[1]=0;
			  	rx_buff[2]=0;

		  }

		  else if(rx_buff[0]=='L')
		  {
			    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
			    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, o);
			    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);
			    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);

			    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
			    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
			    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
			    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);

			  	rx_buff[0]=0;
			  	rx_buff[1]=0;
			  	rx_buff[2]=0;
		  }/*

		  else if(rx_buff[0]=='Q')
		  {
			    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
			    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
			    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);
			    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);

			    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
			    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
			    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
			    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);

			  	rx_buff[0]=0;
			  	rx_buff[1]=0;
			  	rx_buff[2]=0;
		  }

		  else if(rx_buff[0]=='C')
		  {
			    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
			    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
			    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);
			    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);

			    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
			    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
			    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
			    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);

			  	rx_buff[0]=0;
			  	rx_buff[1]=0;
			  	rx_buff[2]=0;
		  }

		  else if(rx_buff[0]=='E')
		  {
			    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
			    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
			    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1);
			    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);

			    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
			    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
			    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
			    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);

			  	rx_buff[0]=0;
			  	rx_buff[1]=0;
			  	rx_buff[2]=0;
		  }

		  else if(rx_buff[0]=='Q')
		  {
			    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
			    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
			    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);
			    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1);

			    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
			    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
			    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
			    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);

			  	rx_buff[0]=0;
			  	rx_buff[1]=0;
			  	rx_buff[2]=0;
		  }*/

		  else if(rx_buff[0]=='X')
		  {
			  HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
			  HAL_Delay(2000);
			  HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
			  speed=2000;
			  	rx_buff[0]=0;
			  	rx_buff[1]=0;
			  	rx_buff[2]=0;

		  }

		  else if(rx_buff[0]=='v')
		  {
			  HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
			  HAL_Delay(3000);
			  HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
			  speed=1000;
			  	rx_buff[0]=0;
			  	rx_buff[1]=0;
			  	rx_buff[2]=0;
		  }

		  else if(rx_buff[0]=='Y')
		  {
			  HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
			  HAL_Delay(1000);
			  HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
		  	  speed=4096;
			  	rx_buff[0]=0;
			  	rx_buff[1]=0;
			  	rx_buff[2]=0;
		  }

		    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, speed);
		    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2, speed);
		    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3, speed);
		    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3, speed);
		    HAL_Delay(50);

	    /* USER CODE BEGIN 3 */
	  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 127;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 625;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 127;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 625;
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
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  huart1.Init.Mode = UART_MODE_RX;
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
  huart2.Init.BaudRate = 9600;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB14 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
