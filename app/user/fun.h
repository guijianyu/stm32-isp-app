#ifndef __FUN_H
#define	__FUN_H

#include "stm32f0xx.h"
#include "types.h"

#define MAX_DUTY	100

#define FUN_PWM_FREQ	25000
#define DEFAULT_DUTY	50
#define TIM2_PRESCALER	100

#define ARRAY_SIZE(x) (sizeof(x)/sizeof((x)[0]))

typedef enum {
	Fun1 = 0,
	Fun2
} Fun_t;

struct temp_rpm{
	uint8_t temp;
	uint16_t rpm;
};

struct rpm_duty{
	uint16_t rpm;
	uint8_t duty;
};

void Fun_PWM_Config(void);
void Fun_Speed_Config(void);
uint8_t PWM_Set_Pulse(Fun_t ft, uint8_t duty);
uint32_t get_rotate_speed(Fun_t ft);
void temp_rpm_regulate(uint8_t env, uint8_t ledr);
void regulate_fun(uint16_t rpm);
void fun_tick(void);

#endif

