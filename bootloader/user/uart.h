#ifndef __UART_H
#define	__UART_H

#include "stm32f0xx.h"
#define countof(a)   (sizeof(a) / sizeof(*(a)))

#define UART_BAUD_RATE 115200

void USART_Config(void);
void USART_Transfer(uint8_t* data, uint8_t len);
#endif

