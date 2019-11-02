#ifndef __LED_H
#define	__LED_H

#include "stm32f0xx.h"
#define LED1_PIN      	GPIO_Pin_14
#define LED1_PORT       GPIOB
#define LED1_CLK		RCC_AHBPeriph_GPIOB

#define LED2_PIN       	GPIO_Pin_15
#define LED2_PORT       GPIOB
#define LED2_CLK		RCC_AHBPeriph_GPIOB

typedef enum{
	LED1 = 0,
	LED2,
}LedNum_t;

void LED_Init(void);
void LED_Open(LedNum_t num);
void LED_Close(LedNum_t num);
void LED_Toggle(LedNum_t num);
#endif /* __LED_H */

