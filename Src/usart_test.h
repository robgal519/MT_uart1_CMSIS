// File: usart_test.h
// Author: Robert Gałat
// Email: robgal519@gmail.com
#include <stdint.h>
#include <stdbool.h>

void init_usart1(void **internal, uint32_t baudrate);
bool test_send(void *internal, uint8_t *data, uint16_t size);
bool test_uninitialize(void *internal);