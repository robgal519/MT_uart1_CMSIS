/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "Driver_USART.h"
#include "stdio.h"
#include "stm32f4xx_hal_rcc.h"
#include "string.h"
#include <stdint.h>
#include <stdlib.h> // random
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

/* USER CODE BEGIN PV */
extern ARM_DRIVER_USART Driver_USART1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint8_t log_buffer[10000];
static uint8_t* log_buffer_head = log_buffer;



void log_message(char* ch){
    while(*ch)
        ITM_SendChar(*ch++);
}

void initUSART(ARM_USART_SignalEvent_t f, unsigned int baudrate, ARM_DRIVER_USART *uart)
{
  uart->Initialize(f);
  uart->PowerControl(ARM_POWER_FULL);
  uart->Control(ARM_USART_MODE_ASYNCHRONOUS |
                    ARM_USART_DATA_BITS_8 |
                    ARM_USART_PARITY_NONE |
                    ARM_USART_STOP_BITS_1 |
                    ARM_USART_FLOW_CONTROL_NONE,
                baudrate);
  uart->Control(ARM_USART_CONTROL_TX, 1);
  uart->Control(ARM_USART_CONTROL_RX, 1);
}
void initGPIOA()
{
  static GPIO_InitTypeDef outputPins;
  outputPins.Pin = GPIO_PIN_4;
  outputPins.Mode = GPIO_MODE_OUTPUT_PP;
  outputPins.Pull = GPIO_PULLDOWN;
  outputPins.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  // outputPins.Alternate  not set

  HAL_GPIO_Init(GPIOA, &outputPins);
}
static uint8_t dummy[500];
void randomizeData(uint8_t *buffor, size_t size)
{
  for (size_t i = 0; i < size; i++)
  {
    buffor[i] = rand() % ((uint8_t)(-1));
  }
}

volatile bool UART1_transfer_Complete = false;

uint32_t eventLog[200];
uint32_t eventLogHead = 0;
void UART_eventHandler(uint32_t event)
{
  // log_buffer_head += sprintf(log_buffer_head, "%lu,",event);
 // eventLog[eventLogHead++] = event;
  if (event & ARM_USART_EVENT_SEND_COMPLETE)
    UART1_transfer_Complete = true;
  // if(event&ARM_USART_EVENT_TX_UNDERFLOW)
  //   UART1_transfer_Complete = true;
}
size_t testBaudrate(size_t baudrate, ARM_DRIVER_USART *uart)
{

  volatile static size_t counter = 0;

  initUSART(UART_eventHandler, baudrate, uart);
  counter = 0;
  UART1_transfer_Complete = false;
  randomizeData(dummy, sizeof(dummy));
  int32_t Status = uart->Send(dummy, sizeof(dummy));
  if (Status != ARM_DRIVER_OK)
  {
    // error occured!!
  }
  else
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    while (!UART1_transfer_Complete)
    {
      counter++;
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  }
  uart->Uninitialize();
  return counter;
}
void printResult(size_t *results, size_t size, ARM_DRIVER_USART *uart)
{
  initUSART(NULL, 9600, uart);
  for (size_t d = 0; d < size; d++)
  {
    static char message[10];
    sprintf(message, "%d|", results[d]);
    uart->Send(message, strlen(message));
    while (uart->GetStatus().tx_busy)
    {
      ;
    }
  }
}

static size_t baudrates[] = {
        4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600,
    1312500,
    2625000,
    5250000,
    // 10500000 not possible with oversampling
    };
#define TEST_COUNT (sizeof(baudrates) / sizeof(*baudrates))
#define RETRY 5
void TEST_CMSIS()
{
  static ARM_DRIVER_USART *uart = &Driver_USART1;

  static size_t results[TEST_COUNT * RETRY] = {0};

  for (size_t test = 0; test < TEST_COUNT; test++)
  {
    for (size_t retry = 0; retry < RETRY; retry++)
    {
      results[test * RETRY + retry] = testBaudrate(baudrates[test], uart);
      char buff[128];
      sprintf(buff,"%d,%d\n", baudrates[test],results[test*RETRY+retry]);
      log_message(buff);
      // log_message(log_buffer);
      // log_message("\n");
      // memset(log_buffer,0,5000);
      // log_buffer_head = log_buffer;
    }
  }
  
  //printResult(results, TEST_COUNT * RETRY, uart);
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  initGPIOA();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  TEST_CMSIS();
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
