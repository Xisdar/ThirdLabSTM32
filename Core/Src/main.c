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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "ctype.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	char TxBuf[50];
} UARTTxQueue_t;
typedef struct {
	char RxBuf[16];
} UARTRxQueue_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_LED 4
#define Shift_Amount 1
#define RxBuf_SIZE 16
#define TxBuf_SIZE 50
#define MAX_TASKS 2
#define NUM_COMMAND 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for LED1Task */
osThreadId_t LED1TaskHandle;
const osThreadAttr_t LED1Task_attributes = {
  .name = "LED1Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for BTNTask */
osThreadId_t BTNTaskHandle;
const osThreadAttr_t BTNTask_attributes = {
  .name = "BTNTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UARTTxTask */
osThreadId_t UARTTxTaskHandle;
const osThreadAttr_t UARTTxTask_attributes = {
  .name = "UARTTxTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UARTRxTask */
osThreadId_t UARTRxTaskHandle;
const osThreadAttr_t UARTRxTask_attributes = {
  .name = "UARTRxTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UARTTxQueue */
osMessageQueueId_t UARTTxQueueHandle;
const osMessageQueueAttr_t UARTTxQueue_attributes = {
  .name = "UARTTxQueue"
};
/* Definitions for UARTRxQueue */
osMessageQueueId_t UARTRxQueueHandle;
const osMessageQueueAttr_t UARTRxQueue_attributes = {
  .name = "UARTRxQueue"
};
/* Definitions for BtnMainSem */
osSemaphoreId_t BtnMainSemHandle;
const osSemaphoreAttr_t BtnMainSem_attributes = {
  .name = "BtnMainSem"
};
/* Definitions for UARTTxSem */
osSemaphoreId_t UARTTxSemHandle;
const osSemaphoreAttr_t UARTTxSem_attributes = {
  .name = "UARTTxSem"
};
/* USER CODE BEGIN PV */
uint8_t RxBuf[RxBuf_SIZE];
// Структура для хранения массивов команд и сообщений об ошибках
struct CommandCMD {
    const char *Name[NUM_COMMAND];
    const char *ErrorMessages[NUM_COMMAND];
    const char *CompletedMessages[NUM_COMMAND];
} CMD = {
    {
    	"f="
    },
    {
        "Invalid command format. Command should be in the format 'F=x.x' with x in the range 0.1 to 9.9 Hz.\n"
    },
    {
        "LED updates frequently: f=%.2f Hz \n"
    }
};
struct State_User_Buttons {
	uint8_t stateUB1 :2;
};
struct Stored_LEDs
{
    uint8_t stateLEDArray1:NUM_LED;
	uint8_t ModeLED1:1;
	uint16_t RefreshRateLED;
} SLED;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void StartLED1Task(void *argument);
void StartBTNTask(void *argument);
void StartUARTTxTask(void *argument);
void StartUARTRxTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Функция для выполнения кольцевого сдвига битов влево и возвращения числа
void circularLeftShiftLEDByte(struct Stored_LEDs* SLED) {
    SLED->stateLEDArray1 = (SLED->stateLEDArray1 << Shift_Amount) | (SLED->stateLEDArray1 >> (4 - Shift_Amount));
}

// Функция для извлечения определенного бита из поля x
uint8_t getLEDByte(struct Stored_LEDs* SLED, uint8_t bitIndex) {
    return (SLED->stateLEDArray1 >> bitIndex) & 1;
}

void ToggleLEDByte(struct Stored_LEDs* SLED)
{
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, getLEDByte(SLED, 3));
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, getLEDByte(SLED, 2));
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, getLEDByte(SLED, 1));
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, getLEDByte(SLED, 0));
}

void Command_Processing(struct CommandCMD* CMD, char *strRx, char *strTx, struct Stored_LEDs* SLED) {
	int commandIndex = -1;

    // Преобразуйте все символы входной строки в нижний регистр для нечувствительности к регистру.
    for (int i = 0; strRx[i]; i++) {
        strRx[i] = tolower(strRx[i]);
    }

    // Найдите индекс соответствующей команды.
    for (uint8_t i = 0; i < NUM_COMMAND; i++) {
        if (strncmp(strRx, CMD->Name[i], strlen(CMD->Name[i])) == 0) {
            commandIndex = i;
            break;
        }
    }

    // Обработка команд на основе найденного индекса.
    switch (commandIndex) {
    	case 0:{
        	char *endPtr;
        	float frequency = strtod(strRx + 2, &endPtr);
        	if (endPtr != strRx + 2 && frequency >= 0.1 && frequency <= 9.9) {
        		SLED->RefreshRateLED = (uint16_t)((1/frequency)*1000);
        		sprintf(strTx, CMD->CompletedMessages[commandIndex], frequency);
            } else strcpy(strTx, CMD->ErrorMessages[commandIndex]);
        } break;

        default:
            // Выведите текст ошибки, если ни одна из команд не совпала.
        	strcpy(strTx, "Invalid command. Supported commands: 'F=x.x'\n");
        break;
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *) RxBuf, RxBuf_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
  SLED.RefreshRateLED = 1000;
  SLED.stateLEDArray1 = 10;

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of BtnMainSem */
  BtnMainSemHandle = osSemaphoreNew(1, 0, &BtnMainSem_attributes);

  /* creation of UARTTxSem */
  UARTTxSemHandle = osSemaphoreNew(1, 0, &UARTTxSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of UARTTxQueue */
  UARTTxQueueHandle = osMessageQueueNew (10, sizeof(UARTTxQueue_t), &UARTTxQueue_attributes);

  /* creation of UARTRxQueue */
  UARTRxQueueHandle = osMessageQueueNew (10, sizeof(UARTRxQueue_t), &UARTRxQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of LED1Task */
  LED1TaskHandle = osThreadNew(StartLED1Task, NULL, &LED1Task_attributes);

  /* creation of BTNTask */
  BTNTaskHandle = osThreadNew(StartBTNTask, NULL, &BTNTask_attributes);

  /* creation of UARTTxTask */
  UARTTxTaskHandle = osThreadNew(StartUARTTxTask, NULL, &UARTTxTask_attributes);

  /* creation of UARTRxTask */
  UARTRxTaskHandle = osThreadNew(StartUARTRxTask, NULL, &UARTRxTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BTN1_Pin */
  GPIO_InitStruct.Pin = BTN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED4_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Press_Fixation_UB1(struct State_User_Buttons* SUB){
	if((SUB->stateUB1 & 0x2) != 0){
		osSemaphoreRelease(BtnMainSemHandle);
		SUB->stateUB1 = 0;
	}
}

void Button_status_change(struct State_User_Buttons* SUB, uint16_t GPIO_Pin){
	uint8_t State = !(HAL_GPIO_ReadPin(BTN1_GPIO_Port, GPIO_Pin));
	SUB->stateUB1 =  SUB->stateUB1 << 1;
	if(State) SUB->stateUB1 |= (State);
	else SUB->stateUB1 &= ~(State);
}

// Функция обратного вызова обработки прерывания EXTI External Interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static struct State_User_Buttons SUB;
	switch (GPIO_Pin) {
		case BTN1_Pin: {
			Button_status_change(&SUB, GPIO_Pin);
			Press_Fixation_UB1(&SUB);
		} break;
		default: break;
	}
}

/* Функцию обратного вызова UART RX IDLE ------------------------------------------*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART2)
	{
		osMessageQueuePut(UARTRxQueueHandle, &RxBuf, 0, 0);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *) RxBuf, RxBuf_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	}
}
/* Функцию обратного вызова UART TX --------------------------------------------*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2){
		if(HAL_UART_GetState (&huart2) != HAL_UART_STATE_BUSY_TX){
			osSemaphoreRelease(UARTTxSemHandle);
		}
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
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartLED1Task */
/**
* @brief Function implementing the LED1Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLED1Task */
void StartLED1Task(void *argument)
{
  /* USER CODE BEGIN StartLED1Task */
  /* Infinite loop */
  for(;;)
  {
	circularLeftShiftLEDByte(&SLED);
	ToggleLEDByte(&SLED);
    osDelay(SLED.RefreshRateLED);
  }
  /* USER CODE END StartLED1Task */
}

/* USER CODE BEGIN Header_StartBTNTask */
/**
* @brief Function implementing the BTNTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBTNTask */
void StartBTNTask(void *argument)
{
  /* USER CODE BEGIN StartBTNTask */
  /* Infinite loop */
  for(;;)
  {
	  if(osSemaphoreAcquire(BtnMainSemHandle, osWaitForever) == osOK){
	    SLED.ModeLED1 ^= 1;
	    SLED.ModeLED1?(SLED.stateLEDArray1 = 9):(SLED.stateLEDArray1 = 10);
	  }
	  osDelay(500);
  }
  /* USER CODE END StartBTNTask */
}

/* USER CODE BEGIN Header_StartUARTTxTask */
/**
* @brief Function implementing the UARTTxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUARTTxTask */
void StartUARTTxTask(void *argument)
{
  /* USER CODE BEGIN StartUARTTxTask */
  UARTTxQueue_t tempTxmsg;
  /* Infinite loop */
  for(;;)
  {
	memset(tempTxmsg.TxBuf, 0, sizeof(tempTxmsg.TxBuf));
	osMessageQueueGet(UARTTxQueueHandle, &tempTxmsg, 0, osWaitForever);
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*) tempTxmsg.TxBuf, strlen(tempTxmsg.TxBuf));
	osSemaphoreAcquire(UARTTxSemHandle, osWaitForever);
	osDelay(1);
  }
  /* USER CODE END StartUARTTxTask */
}

/* USER CODE BEGIN Header_StartUARTRxTask */
/**
* @brief Function implementing the UARTRxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUARTRxTask */
void StartUARTRxTask(void *argument)
{
  /* USER CODE BEGIN StartUARTRxTask */
  UARTRxQueue_t tempRxmsg;
  UARTTxQueue_t tempTxmsg;
  /* Infinite loop */
  for(;;)
  {
    memset(tempRxmsg.RxBuf, 0, sizeof(tempRxmsg.RxBuf));
  	memset(tempTxmsg.TxBuf, 0, sizeof(tempTxmsg.TxBuf));
	osMessageQueueGet(UARTRxQueueHandle, &tempRxmsg, 0, osWaitForever);
	Command_Processing(&CMD, tempRxmsg.RxBuf, tempTxmsg.TxBuf, &SLED);
	if(strlen(tempTxmsg.TxBuf) != 0){
		osMessageQueuePut(UARTTxQueueHandle, &tempTxmsg, 0, osWaitForever);
	}
	osDelay(1);
  }
  /* USER CODE END StartUARTRxTask */
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
