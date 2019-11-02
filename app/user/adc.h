#ifndef __ADC_H
#define	__ADC_H

#include "stm32f0xx.h"
#include "dlp_control.h"

//Temperature and fan speed control strategy definition
#define TEMP_ENV_NODE0		30
#define TEMP_ENV_NODE1		40
#define TEMP_ENV_WARNING		TEMP_ENV_NODE1
#define TEMP_LEDR_WARNING		65
#define TEMP_LEDR_SHUTDOWN		70

#define TEMP_LIMIT_ENV_WARNING		0
#define TEMP_LIMIT_LEDR_WARNING		1
#define TEMP_LIMIT_LEDR_SHUTDOWN	2

//Temperature sensor gpio pin define
#define TEMP_SENSOR_0_PORT		GPIOA
#define TMEP_SENSOR_0_PIN		GPIO_Pin_0
#define TMEP_SENSOR_0_CLK		RCC_AHBPeriph_GPIOA
#define TMEP_SENSOR_0_ADC_CHANNEL ADC_Channel_0

#define TEMP_SENSOR_1_PORT		GPIOA
#define TMEP_SENSOR_1_PIN		GPIO_Pin_1
#define TMEP_SENSOR_1_CLK		RCC_AHBPeriph_GPIOA
#define TMEP_SENSOR_1_ADC_CHANNEL ADC_Channel_1

//Ambient light sensor gpio pin define
#define LIGHT_SENSOR_0_PORT		GPIOA
#define LIGHT_SENSOR_0_PIN		GPIO_Pin_4	
#define LIGHT_SENSOR_0_CLK		RCC_AHBPeriph_GPIOA
#define LIGHT_SENSOR_0_ADC_CHANNEL ADC_Channel_4

#define LIGHT_SENSOR_1_PORT		GPIOA
#define LIGHT_SENSOR_1_PIN		GPIO_Pin_5
#define LIGHT_SENSOR_1_CLK		RCC_AHBPeriph_GPIOA
#define LIGHT_SENSOR_1_ADC_CHANNEL ADC_Channel_5

//clk define
#define ADC_ALL_GPIO_CLK		(TMEP_SENSOR_0_CLK | TMEP_SENSOR_1_CLK | LIGHT_SENSOR_0_CLK | LIGHT_SENSOR_1_CLK | DLP_IADJ_ADC_CLK)

#define TEMP_SENSOR_0		0	//环境温度
#define TEMP_SENSOR_1		1	//光机温度
#define LIGHT_SENSOR_0		0
#define	LIGHT_SENSOR_1		1
	
//function define
void ADC_Config_Init(void);
double get_temperauter_value(int number);
double get_light_value(int number);
double get_dlp_IADJ_voltage(void);
#endif /* __ADC_H */
