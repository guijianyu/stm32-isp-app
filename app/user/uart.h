#ifndef __UART_H
#define	__UART_H

#include "stm32f0xx.h"
#define countof(a)   (sizeof(a) / sizeof(*(a)))
#define SYSTICK_PERIOD	1000	//1ms
#define USER_TIMEOUT	(SYSTICK_PERIOD/10)	//100ms
#define FRAME_RECEIVE_PREPARED		0
#define FRAME_RECEIVE_PROCESSING		1
#define FRAME_RECEIVE_COMPELTED		2

#define UART_BAUD_RATE 115200

void USART_Config(void);
void SysTickConfig(void);
void USART_Transfer(uint8_t* data, uint8_t len);
uint8_t sum(uint8_t* data, uint8_t len);
void USART_Receive(USART_TypeDef* USARTx, uint16_t data);
struct msg* buffer_stat(void);

#endif

