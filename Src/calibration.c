// File: calibration.c
// Author: Robert Gałat
// Email: robgal519@gmail.com

#include "calibration.h"

#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "Driver_USART.h"

extern volatile bool UART_TransferComplete;
void TIM2_IRQHandler(void) {
  // clear interrupt status
  if (TIM2->DIER & 0x01) {
    if (TIM2->SR & 0x01) {
      TIM2->SR &= (uint32_t) ~(1U << 0);
    }
  }
  TIM2->CR1 &= (~((uint16_t)1));
  UART_TransferComplete = true;
  NVIC_DisableIRQ(TIM2_IRQn);
}

void configure_timer(uint32_t baudrate) {
  (void)baudrate;
  RCC->APB1ENR |= (1 << 0);

  // Timer clock runs at ABP1 * 2
  //   since ABP1 is set to /4 of fCLK
  //   thus 168M/4 * 2 = 84Mhz
  // set prescaler to 83999
  //   it will increment counter every prescalar cycles
  // fCK_PSC / (PSC[15:0] + 1)
  // 84 Mhz / 8399 + 1 = 10 khz timer clock speed
  TIM2->PSC = 0;

  // Set the auto-reload value to 10000
  //   which should give 1 second timer interrupts
  TIM2->ARR = 84000000;

  // Update Interrupt Enable
  TIM2->DIER |= (1 << 0);

  NVIC_SetPriority(TIM2_IRQn, 2); // Priority level 2
  // enable TIM2 IRQ from NVIC
  NVIC_EnableIRQ(TIM2_IRQn);
}

int32_t start_timer(const void *data, uint32_t size) {
  (void)data;
  (void)size;
  TIM2->CR1 |= (1 << 0);
  return ARM_DRIVER_OK;
}

uint32_t diable_timer(void) {
  TIM2->CR1 &= (~((uint16_t)1));
  NVIC_DisableIRQ(TIM2_IRQn);
  return ARM_DRIVER_OK;
}
