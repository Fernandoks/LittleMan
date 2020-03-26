/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define KEY_UP				610
#define KEY_DOWN			1550
#define KEY_SELECT			3850
#define KEY_LEFT			2460
#define KEY_RIGHT			0
#define	KEY_ERROR			50
#define KEY_UP_UPPER		KEY_UP + KEY_ERROR
#define KEY_UP_LOWER		KEY_UP - KEY_ERROR
#define KEY_DOWN_UPPER		KEY_DOWN + KEY_ERROR
#define KEY_DOWN_LOWER		KEY_DOWN - KEY_ERROR
#define KEY_LEFT_UPPER		KEY_LEFT + KEY_ERROR
#define KEY_LEFT_LOWER		KEY_LEFT - KEY_ERROR
#define KEY_RIGHT_UPPER		KEY_RIGHT + KEY_ERROR
#define KEY_RIGHT_LOWER		KEY_RIGHT - KEY_ERROR
#define KEY_SELECT_UPPER	KEY_SELECT + KEY_ERROR
#define KEY_SELECT_LOWER	KEY_SELECT - KEY_ERROR

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId Task2Handle;
/* USER CODE BEGIN PV */

osThreadId TaskUARTProcessHandler;
osThreadId TaskDisplayHandler;

osMessageQId ADCQueue;

volatile uint16_t ADCValue = 0;

uint8_t usart_buffer[50] = "START";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void const * argument);
void Task2Init(void const * argument);

/* USER CODE BEGIN PFP */
/*
 * TASKS Definitions
 */
void TaskUARTProcessInit(void const * argument);
void TaskDisplayInit(void const * argument);
int _write(int file, char *ptr, int len);

/*
 * General Functions definitions
 */
void uart_send(uint8_t *data);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */


  /* Initialize LCD 20 cols x 4 rows */
  TM_HD44780_Init(16, 2);
  /* Put string to LCD */
  TM_HD44780_Puts(0, 0, "Fernando");
  TM_HD44780_Puts(0, 1, "16x2 HD44780 LCD");
  /* Wait a little */
  HAL_Delay(3000);
  /* Clear LCD */
  TM_HD44780_Clear();
  /* Write new text */
  TM_HD44780_Puts(6, 1, "CLEARED!");
  /* Wait a little */
  HAL_Delay(1000);

  printf("SVW TEST");

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
  osMessageQDef(adcqueue, 10, uint16_t);
  ADCQueue = osMessageCreate (osMessageQ(adcqueue), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Task2 */
  osThreadDef(Task2, Task2Init, osPriorityNormal, 0, 128);
  Task2Handle = osThreadCreate(osThread(Task2), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  /* TaskUARTProcess Creation */
  osThreadDef(TaskUARTProcess, TaskUARTProcessInit, osPriorityNormal, 0, 128);
  TaskUARTProcessHandler = osThreadCreate(osThread(TaskUARTProcess), NULL);

  /* TaskDisplay Creation */
  osThreadDef(TaskDisplay, TaskDisplayInit, osPriorityNormal, 0, 128);
  TaskDisplayHandler = osThreadCreate(osThread(TaskDisplay), NULL);

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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

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
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

int _write(int file, char *ptr, int len)
{
  /* Implement your write code here, this is used by puts and printf for example */
  int i=0;
  for(i = 0; i < len; i++)
  {
    ITM_SendChar((*ptr++));
  }
  return len;
}

void uart_send(uint8_t *data)
{
	uint16_t size = (uint16_t)strlen((const char *)data);
	HAL_UART_Transmit(&huart2, data, size, HAL_MAX_DELAY);
}


void TaskUARTProcessInit(void const * argument)
{
  for(;;)
  {
    osDelay(1);
  }
}

void TaskDisplayInit(void const * argument)
{
  osEvent event;

  uint16_t ADCValuebefore = 0;
  uint16_t ADCValue = 0;

  HAL_ADC_Start_IT(&hadc1);


  for(;;)
  {


#if 0
	HAL_ADC_PollForConversion(&hadc1, 100);

	if ((HAL_ADC_GetState(&hadc1) & HAL_ADC_STATE_EOC_REG) == HAL_ADC_STATE_EOC_REG)
	{
		ADCValue = HAL_ADC_GetValue(&hadc1);
	}
	else Error_Handler();
#endif

	/*
	 * Description: This part must detect the button press. To do so, it must debounce the button,
	 * and and consider the ADC error.
	 */

	//This get the ADC value from the ADC Queue
	event = osMessageGet(ADCQueue, 100);
	if(event.status == osEventMessage)
	{
		ADCValue = event.value.v;
	}

	if( !((ADCValue > (ADCValuebefore + KEY_ERROR)) && (ADCValue < (ADCValuebefore - KEY_ERROR))) )
	{

		if ( (ADCValue > KEY_UP_LOWER) && (ADCValue < KEY_UP_UPPER) )
		{
			sprintf(usart_buffer,"UP\r\n");
			uart_send(usart_buffer);
			TM_HD44780_Clear();
			TM_HD44780_Puts(0, 0, "UP");
		}
		else if (( (ADCValue > KEY_DOWN_LOWER) && (ADCValue < KEY_DOWN_UPPER) ))
		{
			sprintf(usart_buffer,"DOWN\r\n");
			uart_send(usart_buffer);
			TM_HD44780_Clear();
			TM_HD44780_Puts(0, 0, "DOWN");
		}
		else if (( (ADCValue > KEY_RIGHT_LOWER) && (ADCValue < KEY_RIGHT_UPPER) ))
		{
			sprintf(usart_buffer,"RIGHT\r\n");
			uart_send(usart_buffer);
			TM_HD44780_Clear();
			TM_HD44780_Puts(0, 0, "RIGHT");
		}
		else if (( (ADCValue > KEY_LEFT_LOWER) && (ADCValue < KEY_LEFT_UPPER) ))
		{
			sprintf(usart_buffer,"LEFT\r\n");
			uart_send(usart_buffer);
			TM_HD44780_Clear();
			TM_HD44780_Puts(0, 0, "LEFT");
		}
		else if (( (ADCValue > KEY_SELECT_LOWER) && (ADCValue < KEY_SELECT_UPPER) ))
		{
			sprintf(usart_buffer,"SELECT\r\n");
			uart_send(usart_buffer);
			TM_HD44780_Clear();
			TM_HD44780_Puts(0, 0, "SELECT");
		}

		ADCValuebefore = ADCValue;
	}

	HAL_ADC_Start(&hadc1);
    osDelay(10);
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1)
	{

		uint16_t ADCValue = HAL_ADC_GetValue(hadc);
		if(osMessagePut (ADCQueue, ADCValue, 100) != osOK)
		{
		  Error_Handler();
		}
		printf("ADC = %d",ADCValue);
	}

}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_Task2Init */
/**
* @brief Function implementing the Task2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task2Init */
void Task2Init(void const * argument)
{
  /* USER CODE BEGIN Task2Init */
  /* Infinite loop */
  for(;;)
  {

    osDelay(1);
  }
  /* USER CODE END Task2Init */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
