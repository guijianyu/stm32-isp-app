#ifndef __STEPMOTOR_H
#define	__STEPMOTOR_H

#include "stm32f0xx.h"
#include "uart.h"
//A~AIN1~PA13, AIN2~PA12, BIN2~PA14, BIN1~PA15
#define PIN_A	GPIO_Pin_13
#define PIN_AN	GPIO_Pin_12
#define PIN_B	GPIO_Pin_15
#define PIN_BN	GPIO_Pin_14

//M_SLEEP~PB3
#define PIN_SLEEP	GPIO_Pin_3

//MFAULT~PB4, MOTOR_SENSOR~PB5
#define PIN_FAULT	GPIO_Pin_4
#define PIN_SENSOR	GPIO_Pin_5

//步进电机，二线四拍
#define PHASES	4
#define POLARIY	2

#define	BEAT_TICKS	(SYSTICK_PERIOD/50)	//20ms

typedef enum {
	CounterClockWise = 0,		//counter clock wise,面朝光机后侧，右转
	ClockWise	//clock wise,面朝光机后侧，左转
} Rotate_t;

typedef enum {
	N = -1,		//Negative
	Z,		//Neutrl
	P	//Positive
} Phase_t;

void StepMotor_Config(void);
uint8_t SM_Rotate(Rotate_t rt, uint32_t bts);
void beat(void);
uint8_t SM_Fault(void);
uint8_t SM_Limit(void);
void SM_Sleep(void);
void SM_Wake(void);

#endif

