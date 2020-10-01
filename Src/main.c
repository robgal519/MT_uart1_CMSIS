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

#include "calibration.h"
#include "uart_test.h"

#include <stdbool.h>
#include <stdlib.h> // random
#include <stdint.h>
#include "stdio.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stm32f4xx_hal_rcc.h"
#include "string.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void log_message(char *ch) {
  while (*ch)
    ITM_SendChar(*ch++);
}


void randomize_payload(uint8_t *buffor, size_t size) {
  for (size_t i = 0; i < size; i++) {
    buffor[i] = rand() % ((uint8_t)(-1));
  }
}

volatile bool UART_TransferComplete;
struct test_ctx {
  void (*configure)(uint32_t baudrate);
  int32_t (*transfer)(const void *data, uint32_t size);
  uint32_t (*deinit)(void);
};


struct test_ctx calibration = {
    .configure = configure_timer,
    .transfer = start_timer,
    .deinit = diable_timer,
};

struct test_ctx test = {
    .configure = init_usart1,
    .transfer = test_send,
    .deinit = test_uninitialize,
};

bool test_performance(struct test_ctx *ctx, uint32_t baud, uint32_t *counter) {

  static uint8_t data[500];
  uint32_t cnt = 0;

  if (ctx == NULL)
    return false;

  if (ctx->configure == NULL)
    return false;
  ctx->configure(baud);

  randomize_payload(data, sizeof(data));

  if (ctx->transfer == NULL)
    return false;

  UART_TransferComplete = false;

  ctx->transfer(data, sizeof(data));
  GPIOA->ODR |= 1 << 4;
  while (!UART_TransferComplete) {
    cnt++;
  }
  GPIOA->ODR &= (uint32_t) ~(1 << 4);

  if (ctx->deinit == NULL)
    return false;
  ctx->deinit();

  *counter = cnt;
  return true;
}

static uint32_t baudrates[] = {
    4800,   9600,   19200,  38400,   57600,   115200,
    230400, 460800, 921600, 1312500, 2625000, 5250000,
    // 10500000 not possible with oversampling
};
#define TEST_COUNT (sizeof(baudrates) / sizeof(*baudrates))
#define REPS 5

void TEST_CMSIS() {
  char log_buffer[128];
  uint32_t calibration_counter;
  test_performance(&calibration, 0, &calibration_counter);
  sprintf(log_buffer, "Calibration: %lu\n", calibration_counter);
  log_message(log_buffer);

  for (uint32_t boudrate_id = 0; boudrate_id < TEST_COUNT; boudrate_id++) {
    for (uint8_t retry = 0; retry < REPS; retry++) {
      uint32_t counter = 0;
      test_performance(&test, baudrates[boudrate_id], &counter);
      sprintf(log_buffer, "%lu,%lu\n", baudrates[boudrate_id], counter);
      log_message(log_buffer);
    }
  }
}
/* USER CODE END 0 */


void initGPIOA() {
  static GPIO_InitTypeDef outputPins;
  outputPins.Pin = GPIO_PIN_4;
  outputPins.Mode = GPIO_MODE_OUTPUT_PP;
  outputPins.Pull = GPIO_PULLDOWN;
  outputPins.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  // outputPins.Alternate  not set

  HAL_GPIO_Init(GPIOA, &outputPins);
}

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
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
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {

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
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line)
   */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
