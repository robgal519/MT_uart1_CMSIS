// File: calibration.h
// Author: Robert Gałat
// Email: robgal519@gmail.com

#include <stdint.h>

void configure_timer(uint32_t baudrate);
int32_t start_timer(const void *data, uint32_t size);
uint32_t diable_timer(void);