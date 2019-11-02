#ifndef __LED_H
#define	__LED_H

#include "stm32f0xx.h"
#define LED1_PIN      GPIO_Pin_5
//#define LED2_PIN       GPIO_Pin_12
#define LED1_PORT       GPIOB
//#define LED2_PORT       GPIOA
void LED_Init(void);
void LED_Open(void);
void LED_Close(void);
void LED_Toggle(void);
#endif /* __LED_H */

