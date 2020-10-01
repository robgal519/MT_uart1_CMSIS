// File: usart_test.h
// Author: Robert Gałat
// Email: robgal519@gmail.com
#include <stdint.h>

void init_usart1(uint32_t baudrate);
int32_t test_send(const void *data, uint32_t size);
uint32_t test_uninitialize(void);