// File: usart_test.c
// Author: Robert Gałat
// Email: robgal519@gmail.com

#include "usart_test.h"

#include "Driver_USART.h"

extern ARM_DRIVER_USART Driver_USART1;

extern volatile bool UART_TransferComplete;

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
  if (event & ARM_USART_EVENT_SEND_COMPLETE)
    UART_TransferComplete = true;
}
void init_usart1(void **internal, uint32_t baudrate) {
  initUSART(UART_eventHandler, baudrate, &Driver_USART1);
  *internal = (void *)&Driver_USART1;
}

bool test_send(void *internal, uint8_t *data, uint16_t size) {
  if (internal != &Driver_USART1)
    return false;
  Driver_USART1.Send(data, size);
  return true;
}

bool test_uninitialize(void *internal) {
  if (internal != &Driver_USART1)
    return false;
  Driver_USART1.Uninitialize();
  return true;
}