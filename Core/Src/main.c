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
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "ctype.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_LED 4
#define Shift_Amount 1
#define RxBuf_SIZE 16
#define TxBuf_SIZE 128
#define MAX_TASKS 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
struct Stored_LEDs
{
    uint8_t LEDArray1:NUM_LED;
	uint8_t ModeLED1:1;
    uint8_t RefreshRateLED;
} SLED;

struct State_User_Buttons
{
	uint8_t stateUB1:2;
	uint8_t flagUB1:1;
} SUB;

typedef struct {
    void (*taskFunction)(void);  // Указатель на функцию, которую нужно выполнить.
    uint32_t period;             // Период выполнения задачи (в миллисекундах).
    uint32_t lastExecutionTime;  // Время последнего выполнения задачи (в миллисекундах).
} Task;

// Массив поддерживаемых команд и соответствующих текстов ошибок.
const char *commands[] = { "f="};
const char *errorMessages[] = {
    "Invalid command format. Command should be in the format 'F=x.x' with x in the range 0.1 to 9.9 Hz.\n"
};
const char *completedMessages[] = {
	"LED updates frequently: f=%.2f Hz\n"
};
uint8_t TxBuf[TxBuf_SIZE];
uint8_t RxBuf[RxBuf_SIZE];
Task tasks[MAX_TASKS];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Функция для выполнения кольцевого сдвига битов влево и возвращения числа
void circularLeftShiftLEDByte(struct Stored_LEDs* SLED) {
    SLED->LEDArray1 = (SLED->LEDArray1 << Shift_Amount) | (SLED->LEDArray1 >> (4 - Shift_Amount));
}

// Функция для извлечения определенного бита из поля x
uint8_t getLEDByte(struct Stored_LEDs* SLED, uint8_t bitIndex) {
    return (SLED->LEDArray1 >> bitIndex) & 1;
}

void ToggleLEDByte(struct Stored_LEDs* SLED)
{
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, getLEDByte(SLED, 3));
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, getLEDByte(SLED, 2));
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, getLEDByte(SLED, 1));
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, getLEDByte(SLED, 0));
}

void task1Function(void) {
	circularLeftShiftLEDByte(&SLED);
	ToggleLEDByte(&SLED);
}

void task2Function(void) {
    if(SUB.flagUB1 == 1){
    	SLED.ModeLED1 ^= 1;
    	SLED.ModeLED1?(SLED.LEDArray1 = 8):(SLED.LEDArray1 = 10);
    	SUB.flagUB1 = 0;
    	task1Function();
    }
}

void TaskScheduler(Task* tasks, int numTasks) {
    while (1) {
        for (int i = 0; i < numTasks; i++) {
            if (HAL_GetTick() - tasks[i].lastExecutionTime >= tasks[i].period) {
                tasks[i].taskFunction();  // Выполнить задачу.
                tasks[i].lastExecutionTime = HAL_GetTick();  // Обновить время последнего выполнения.
            }
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *) RxBuf, RxBuf_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
  SLED.RefreshRateLED = 10;
  SLED.LEDArray1 = 10;

  // Инициализация микроконтроллера и других компонентов.
  tasks[0].taskFunction = task1Function;
  tasks[0].period = 1000;
  tasks[0].lastExecutionTime = HAL_GetTick();

  tasks[1].taskFunction = task2Function;
  tasks[1].period = 500;
  tasks[1].lastExecutionTime = HAL_GetTick();

  // Запустить диспетчер задач.
  TaskScheduler(tasks, 2);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
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
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
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
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Press_Fixation_UB1(struct State_User_Buttons* SUB){
	if((SUB->stateUB1 & 0x2) != 0){
		SUB->flagUB1 = 1;
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
	switch (GPIO_Pin) {
		case BTN1_Pin: {
			Button_status_change(&SUB, GPIO_Pin);
			Press_Fixation_UB1(&SUB);
		} break;
		default: break;
	}
}

/* Функцию UART TX --------------------------------------------------------------*/
void UART_TX_Triger(char *strTx){
    if (strlen(strTx) != 0) {
        HAL_UART_Transmit_DMA(&huart2, (uint8_t*)strTx, strlen(strTx));
    }
}

void String_Comp(char *strRx, char *strTx, struct Stored_LEDs* SLED) {
	int commandIndex = -1;

    // Преобразуйте все символы входной строки в нижний регистр для нечувствительности к регистру.
    for (int i = 0; strRx[i]; i++) {
        strRx[i] = tolower(strRx[i]);
    }

    size_t numCommands = sizeof(commands) / sizeof(commands[0]);

    // Найдите индекс соответствующей команды.
    for (size_t i = 0; i < numCommands; i++) {
        if (strncmp(strRx, commands[i], strlen(commands[i])) == 0) {
            commandIndex = i;
            break;
        }
    }

    // Обработка команд на основе найденного индекса.
    switch (commandIndex) {
    	case 0:{
        	char *endPtr;
        	double frequency = strtod(strRx + 2, &endPtr);
        	if (endPtr != strRx + 2 && frequency >= 0.1 && frequency <= 9.9) {
        		tasks[0].period = (uint16_t)(frequency*1000);
        		sprintf(strTx, completedMessages[commandIndex], frequency);
            return;
            } else strcat(strTx, errorMessages[commandIndex]);
        } break;

        default:
            // Выведите текст ошибки, если ни одна из команд не совпала.
        	strcat(strTx, "Invalid command. Supported commands: 'F=x.x'\n");
        break;
    }
}

/* Функцию обратного вызова UART RX IDLE ------------------------------------------*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART2)
	{
		String_Comp((char*)RxBuf,(char*)TxBuf,&SLED);
		UART_TX_Triger((char*)TxBuf);
		memset (RxBuf, 0, RxBuf_SIZE);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *) RxBuf, RxBuf_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	}
}

/* Функцию обратного вызова UART TX --------------------------------------------*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2){
		if(HAL_UART_GetState (&huart2) != HAL_UART_STATE_BUSY_TX){
			memset (TxBuf, 0, TxBuf_SIZE);
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
