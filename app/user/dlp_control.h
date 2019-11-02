#ifndef __DLP_CONTROL_H
#define	__DLP_CONTROL_H

#include "stm32f0xx.h"

#define DLP_GREEN_LIGHT_PWM_PORT			GPIOB
#define DLP_GREEN_LIGHT_PWM_PIN				GPIO_Pin_0
#define DLP_GREEN_LIGHT_PWM_TIM				TIM3
#define DLP_GREEN_LIGHT_PWM_TIM_CHANNEL		TIM_Channel_3
#define DLP_GREEN_LIGHT_PWM_AF				GPIO_AF_1
#define DLP_GREEN_LIGHT_PWM_FREQ			100000 
#define DLP_GREEN_LIGHT_PWM_DUTY_CYCLE		5000

#define DLP_SDIM_PWM_PORT					GPIOB
#define DLP_SDIM_PWM_PIN					GPIO_Pin_1
#define DLP_SDIM_PWM_TIM					TIM3
#define DLP_SDIM_PWM_TIM_CHANNEL			TIM_Channel_4
#define DLP_SDIM_PWM_AF						GPIO_AF_1
#define DLP_SDIM_PWM_FREQ					100000 
#define DLP_SDIM_PWM_DUTY_CYCLE				5000

/* PWM输入捕获引脚 */
#define DLP_ICPWM_GPIO_PORT      		 	GPIOA    
#define DLP_ICPWM_PIN             			GPIO_Pin_6            
#define DLP_ICPWM_AF						GPIO_AF_5
#define DLP_ICPWM_TIM           		    TIM16
#define DLP_ICPWM_CHANNEL       			TIM_Channel_1
#define DLP_ICPWM_SAMPLING_CLK				24000000
/* 捕获/比较中断 */
#define DLP_TIM_IRQn					    TIM16_IRQn
#define DLP_TIM_IRQHandler        			TIM16_IRQHandler
#define DLP_TIM_IT_Event					TIM_IT_CC1

//PB_LED_PWM_IADJ_FB gpio pin define
#define DLP_IADJ_ADC_PORT					GPIOA
#define DLP_IADJ_ADC_PIN					GPIO_Pin_6
#define DLP_IADJ_ADC_CLK					RCC_AHBPeriph_GPIOA
#define DLP_IADJ_ADC_CHANNEL 				ADC_Channel_6
        
/*      
	var define
*/
extern __IO uint32_t g_DlpFrequency;
extern uint16_t g_dlp_out_pwm_TIM_Period;

/*
	function define
*/
uint32_t bsp_GetRCCofGPIO(GPIO_TypeDef* GPIOx);
uint32_t bsp_GetRCCofTIM(TIM_TypeDef* TIMx);
uint8_t  bsp_GetPinSource(uint16_t gpio_pin);
void bsp_ConfigTimGpio(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pinx, uint8_t GPIO_AF_x);
void bsp_ConfigGpioOut(GPIO_TypeDef* GPIOx, uint16_t GPIO_PinX);
void bsp_SetTIMOutPWM(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t GPIO_AF_x, TIM_TypeDef* TIMx, uint8_t _ucChannel,
	 uint32_t _ulFreq, uint32_t _ulDutyCycle);
uint16_t bsp_SelectPWMTrigger(uint8_t _ucChannel, uint16_t TIM_ICSelection);/*Select the TIMx PWM Input Trigger*/
void bsp_SetTIMInPWM(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t GPIO_AF_x, TIM_TypeDef* TIMx, uint8_t _ucChannel,
	uint8_t TIMx_IRQn,	uint16_t TIMx_IT_Event);

void DLP_Init(void);
void DLP_Set_Pulse(TIM_TypeDef* TIMx, uint8_t _ucChannel,  uint16_t usPeriod, uint32_t _ulDutyCycle);
void DLP_TIM_IRQHandler(void);
#endif  //__DLP_CONTROL_H
