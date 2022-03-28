/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l4s5i_iot01.h"
#include "stm32l4s5i_iot01_tsensor.h"
#include "stm32l4s5i_iot01_psensor.h"
#include "stm32l4s5i_iot01_hsensor.h"
#include "stm32l4s5i_iot01_magneto.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
union sensor_data {
	float rd0;
	int16_t rd1[3];
};

typedef struct {
	int sensor_type;
	union sensor_data data;
} sensor_reading;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUTTON_PRESS_SIGNAL 0x00000001
#define ITM_Port32(n) (*((volatile unsigned long *)(0xE0000000+4*n)))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;

osThreadId taskButtonHandle;
osThreadId taskSensorReadHandle;
osThreadId taskSerialWriteHandle;
osMutexId curSensorMutexHandle;
/* USER CODE BEGIN PV */
int cur_sensor = 0;

osMailQDef (sensorQ, 3, sensor_reading);
osMailQId sensorQ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
void StartButtonTask(void const * argument);
void StartSensorRead(void const * argument);
void StartSerialWrite(void const * argument);

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
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  cur_sensor = 0;
  char uart_buffer[100];
    memset(uart_buffer, 0, 100);

    uint32_t temp_status = BSP_TSENSOR_Init();

    if (temp_status == 0) {
  	  strcpy(uart_buffer, "The temperature sensor is successfully initialised.\r\n");
    } else {
  	  strcpy(uart_buffer, "Initialisation for the temperature sensor failed.\r\n");
    }
    HAL_UART_Transmit(&huart1, (uint8_t*) uart_buffer, 100, 20);

    uint32_t humid_status = BSP_HSENSOR_Init();
    memset(uart_buffer, 0, 100);
    if (humid_status == 0) {
  	  strcpy(uart_buffer, "The humidity sensor is successfully initialised.\r\n");
    } else {
  	  strcpy(uart_buffer, "Initialisation for the humidity sensor failed.\r\n");
    }
    HAL_UART_Transmit(&huart1, (uint8_t*) uart_buffer, 100, 20);

    uint32_t pres_status = BSP_PSENSOR_Init();
    memset(uart_buffer, 0, 100);
    if (pres_status == 0) {
  	  strcpy(uart_buffer, "The pressure sensor is successfully initialised.\r\n");
    } else {
  	  strcpy(uart_buffer, "Initialisation for the pressure sensor failed.\r\n");
    }
    HAL_UART_Transmit(&huart1, (uint8_t*) uart_buffer, 100, 20);

    MAGNETO_StatusTypeDef mag_status = BSP_MAGNETO_Init();
    memset(uart_buffer, 0, 100);
    if (mag_status == MAGNETO_OK) {
  	  strcpy(uart_buffer, "The magneto sensor is successfully initialised.\r\n");
    } else {
  	  strcpy(uart_buffer, "Initialisation for the magneto sensor failed.\r\n");
    }
    HAL_UART_Transmit(&huart1, (uint8_t*) uart_buffer, 100, 20);
  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of curSensorMutex */
  osMutexDef(curSensorMutex);
  curSensorMutexHandle = osMutexCreate(osMutex(curSensorMutex));

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
  osMailQDef(sensorQ, 2, sensor_reading);
  sensorQ = osMailCreate(osMailQ(sensorQ), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of taskButton */
  osThreadDef(taskButton, StartButtonTask, osPriorityNormal, 0, 128);
  taskButtonHandle = osThreadCreate(osThread(taskButton), NULL);

  /* definition and creation of taskSensorRead */
  osThreadDef(taskSensorRead, StartSensorRead, osPriorityAboveNormal, 0, 128);
  taskSensorReadHandle = osThreadCreate(osThread(taskSensorRead), NULL);

  /* definition and creation of taskSerialWrite */
  osThreadDef(taskSerialWrite, StartSerialWrite, osPriorityHigh, 0, 256);
  taskSerialWriteHandle = osThreadCreate(osThread(taskSerialWrite), NULL);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c2.Init.Timing = 0x307075B1;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_13) {
		// set signal if button is pressed
		osSignalSet(taskButtonHandle, BUTTON_PRESS_SIGNAL);
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartButtonTask */
/**
  * @brief  Function implementing the taskButton thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartButtonTask */
void StartButtonTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  osDelay(100);
	  osEvent event = osSignalWait(BUTTON_PRESS_SIGNAL, osWaitForever);
	  if (event.status == osEventSignal) {
		  // button is pressed, change sensor
		  osStatus m_status = osMutexWait(curSensorMutexHandle, osWaitForever);
		  if (m_status == osOK){
			  cur_sensor++;
			  if (cur_sensor >= 4) {
				  cur_sensor = 0;
			  }

		  }
		  osMutexRelease(curSensorMutexHandle);
	  }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSensorRead */
/**
* @brief Function implementing the taskSensorRead thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorRead */
void StartSensorRead(void const * argument)
{
  /* USER CODE BEGIN StartSensorRead */
	sensor_reading *reading_ptr;
	osStatus m_status;
  /* Infinite loop */
  for(;;)
  {
	  osDelay(100);

	  reading_ptr = osMailAlloc(sensorQ, osWaitForever);
	  ITM_Port32(0) = (uint32_t) reading_ptr;
	  if (reading_ptr != NULL) {
		  memset(reading_ptr, 0, sizeof(sensor_reading));
		  m_status = osMutexWait(curSensorMutexHandle, 50);
		  if (m_status == osOK) {
			  reading_ptr->sensor_type = (int)cur_sensor;

			  switch (reading_ptr->sensor_type) {
			  case 0: {
				  // temp
				  reading_ptr->data.rd0 = BSP_TSENSOR_ReadTemp();
			  } break;

			  case 1: {
				  // humidity
				  reading_ptr->data.rd0 = BSP_HSENSOR_ReadHumidity();
			  } break;

			  case 2: {
				  // pressure
				  reading_ptr->data.rd0 = BSP_PSENSOR_ReadPressure();
			  } break;

			  case 3: {
				  // magneto
				  BSP_MAGNETO_GetXYZ(reading_ptr->data.rd1);
			  } break;

			  default: {
				  cur_sensor = 0;
			  }
			  }
			  osMailPut(sensorQ, reading_ptr);
		  }
		  osMutexRelease(curSensorMutexHandle);
	  }
  }
  /* USER CODE END StartSensorRead */
}

/* USER CODE BEGIN Header_StartSerialWrite */
/**
* @brief Function implementing the taskSerialWrite thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSerialWrite */
void StartSerialWrite(void const * argument)
{
  /* USER CODE BEGIN StartSerialWrite */
	sensor_reading *reading_ptr;
	osEvent evt;
	char str_buffer[64];
  /* Infinite loop */
  for(;;)
  {
	  // osDelay(10);
	  evt = osMailGet(sensorQ, osWaitForever);
	  ITM_Port32(6) = 4;
	  if (evt.status == osEventMail) {
		  reading_ptr = evt.value.p;
		  ITM_Port32(1) = (uint32_t) reading_ptr;
		  memset(str_buffer, 0, 64);
		  switch (reading_ptr->sensor_type) {
		  case 0: {
			  // temp
			  sprintf(str_buffer, "Temperature: %f \r\n", (reading_ptr->data.rd0));
		  } break;

		  case 1: {
			  // humidity
			  sprintf(str_buffer, "Humidity: %f \r\n", (reading_ptr->data.rd0));
		  } break;

		  case 2: {
			  // pressure
			  sprintf(str_buffer, "Pressure: %f \r\n", (reading_ptr->data.rd0));
		  } break;

		  case 3: {
			  // magneto
			  sprintf(str_buffer, "Magneto: X=%d\tY=%d\tZ=%d \r\n", (int)(reading_ptr->data.rd1[0]), (int)(reading_ptr->data.rd1[1]), (int)(reading_ptr->data.rd1[2]));
		  } break;
		  }
		  HAL_UART_Transmit(&huart1, (uint8_t*) str_buffer, 64, 100);
		  osMailFree(sensorQ, reading_ptr);
	  }
  }
  /* USER CODE END StartSerialWrite */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17) {
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
