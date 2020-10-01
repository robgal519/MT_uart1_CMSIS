// File: uart_test.c
// Author: Robert Gałat
// Email: robgal519@gmail.com

#include "uart_test.h"

#include "Driver_USART.h"

extern ARM_DRIVER_USART Driver_USART1;

volatile bool uart1_transfer_complete = false;

void initUSART(ARM_USART_SignalEvent_t f, unsigned int baudrate,
               ARM_DRIVER_USART *uart) {
  uart->Initialize(f);
  uart->PowerControl(ARM_POWER_FULL);
  uart->Control(ARM_USART_MODE_ASYNCHRONOUS | ARM_USART_DATA_BITS_8 |
                    ARM_USART_PARITY_NONE | ARM_USART_STOP_BITS_1 |
                    ARM_USART_FLOW_CONTROL_NONE,
                baudrate);
  uart->Control(ARM_USART_CONTROL_TX, 1);
  uart->Control(ARM_USART_CONTROL_RX, 1);
}
void UART_eventHandler(uint32_t event) {
  // log_buffer_head += sprintf(log_buffer_head, "%lu,",event);
  // eventLog[eventLogHead++] = event;
  if (event & ARM_USART_EVENT_SEND_COMPLETE)
    uart1_transfer_complete = true;
  // if(event&ARM_USART_EVENT_TX_UNDERFLOW)
  //   UART1_transfer_Complete = true;
}
void init_usart1(uint32_t baudrate) {
  initUSART(UART_eventHandler, baudrate, &Driver_USART1);
}

int32_t test_send(const void *data, uint32_t size) {
  return Driver_USART1.Send(data, size);
}

uint32_t test_uninitialize(void) { return Driver_USART1.Uninitialize(); }