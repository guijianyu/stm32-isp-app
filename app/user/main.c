/**
  ******************************************************************************
  * @file    main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    23-March-2012
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "led.h"
#include "fun.h"
#include "uart.h"
#include "stepmotor.h"
#include "adc.h"
#include "flash_if.h"
#include "types.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "iso_I2C_gpio.h"
#include "iso_I2C_gpio_ee.h"
#include "dlp_control.h"
#include "gpio.h"
#include "bsp_key.h"
#include "xfm10213_model.h"
#include "IWDG_model.h"
#include "bsp_delay.h"

/** @addtogroup STM32F0-Discovery_Demo
  * @{
  */
#define USER_ADDRESS     (uint32_t)0x08004000	//本程序所在flash地址

#if   (defined ( __CC_ARM ))
 // __IO uint32_t VectorTable[48] __attribute__((at(0x20000000)));
#elif (defined (__ICCARM__))
#pragma location = 0x20000000
  __no_init __IO uint32_t VectorTable[48];
#elif defined   (  __GNUC__  )
  __IO uint32_t VectorTable[48] __attribute__((section(".RAMVectorTable")));
#elif defined ( __TASKING__ )
  __IO uint32_t VectorTable[48] __at(0x20000000);
#endif


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern __IO uint32_t BeatCycle; 
extern __IO Rotate_t rotate;	//转向
extern __IO uint32_t beats;	//总节拍数
extern __IO uint32_t beatCount;	//当前节拍

extern __IO uint32_t TIM2CHFreq[];
__IO uint8_t warning = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void process(struct msg* m);
/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
#if 0
	uint32_t i = 0;
	/* Relocate by software the vector table to the internal SRAM at 0x20000000 ***/  

	/* Copy the vector table from the Flash (mapped at the base of the application
	 load address 0x08003000) to the base address of the SRAM at 0x20000000. */
	for(i = 0; i < 48; i++)
		VectorTable[i] = *(__IO uint32_t*)(USER_ADDRESS + (i<<2));

	
	/* Enable the SYSCFG peripheral clock*/
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SYSCFG, ENABLE); 
	/* Remap SRAM at 0x00000000 */
	SYSCFG_MemoryRemapConfig(SYSCFG_MemoryRemap_SRAM);
	  
	__enable_irq();
#endif 	
	LED_Init();
	LED_Close(LED1);    /* 关闭绿灯 */		
	LED_Open(LED2);		/* 开启红灯 */
	
	bsp_InitKey();
	GPIO_Pin_Init();
	USART_Config();
	FLASH_If_Init();	
	SysTickConfig();
	
	I2C_GPIO_Config();
	xfm_init();
	
	StepMotor_Config();
	Fun_PWM_Config();
	Fun_Speed_Config();
	ADC_Config_Init();
	DLP_Init();

	bsp_InitIwdg(3000); /*初始化看门狗为3s*/
	//printf("whh 2019-10-15\r\n");

	while(1)
	{
		/* Reload IWDG counter */
		IWDG_Feed();
		
		process(buffer_stat());
		
		temp_rpm_regulate(get_temperauter_value(TEMP_SENSOR_0), get_temperauter_value(TEMP_SENSOR_1));
		//如果warning出现告警，拉低某个gpio，尚未确定哪根gpio
		
		/*按键唤醒,事件处理*/
		bsp_key_event_process(); 

		/* 循环处理事件 */
		Stm32_Wake_Mstar648_Chip_Loop();
		Stm32_Reset_XFM10213_Chip_Loop();
		Stm32_Reset_I94113_Chip_Loop();		

		#ifdef  DEBUG_TEST
//		uint16_t i = 0;
//		xfm_ReadReg(XFM_MODE_REG, &i);
//		printf("xfm  XFM_MODE_REG data = %#x \r\n",i);
//		xfm_ReadReg(0x010A, &i);
//		printf("xfm  system ver = %#x \r\n",i);	

//		for(i=0;i<=10;i++)
//		{
//			PWM_Set_Pulse(Fun1,i*10);			
//			PWM_Set_Pulse(Fun2,i*10);
//			bsp_DelayMS(10000);
//			printf("FUN%d duty=%d, speed = %d\r\n",Fun1,i*10,get_rotate_speed(Fun1));
//			printf("FUN%d duty=%d, speed = %d\r\n",Fun2,i*10,get_rotate_speed(Fun2));
//			bsp_DelayMS(1000);
//		}
		/*test, 如果延时大于3s, 需要关闭看门狗*/
//		DLP_Set_Pulse(DLP_GREEN_LIGHT_PWM_TIM,DLP_GREEN_LIGHT_PWM_TIM_CHANNEL,g_dlp_out_pwm_TIM_Period,0);
//		bsp_DelayMS(5000);
//		DLP_Set_Pulse(DLP_GREEN_LIGHT_PWM_TIM,DLP_GREEN_LIGHT_PWM_TIM_CHANNEL,g_dlp_out_pwm_TIM_Period,8000);
//		bsp_DelayMS(5000);
//		DLP_Set_Pulse(DLP_GREEN_LIGHT_PWM_TIM,DLP_GREEN_LIGHT_PWM_TIM_CHANNEL,g_dlp_out_pwm_TIM_Period,4000);
//		bsp_DelayMS(5000);
//		get_temperauter_value(TEMP_SENSOR_0);
//		bsp_DelayMS(2000);
//		get_temperauter_value(TEMP_SENSOR_1);
//		bsp_DelayMS(2000);
//		get_light_value(LIGHT_SENSOR_0);
//		bsp_DelayMS(2000);
//		get_light_value(LIGHT_SENSOR_1);
//		bsp_DelayMS(2000);
//		get_dlp_IADJ_voltage();
//		bsp_DelayMS(2000);
		#endif
	}
}

/**
  * @brief  协议内容解析和处理
  * @param  
  * @retval 
  */
void process(struct msg* m)
{
	struct msg res;
	Fun_t fun;

	if(m == NULL)
		return;
	
	memset(&res, 0xFF, BUFFERSIZE);
	res.cmd = m->cmd;
	res.sn = m->sn;
	
	if((m->cmd == CMD_PWM) || (m->cmd == CMD_FUNSPEED))
		fun = m->data[0]? Fun2 :Fun1;
	
	//if(m->check != sum((uint8_t*)m, BUFFERSIZE - 1)){
	//	memset(res.data, 0xFF, BUFFERSIZE -3);
	//	goto transfer;
	//}

	switch(m->cmd){
	case CMD_PWM:
		res.data[0] = PWM_Set_Pulse(fun, m->data[1]);
		break;
	case CMD_FUNSPEED:
		res.data[0] = m->data[0];
		res.data[1] = (get_rotate_speed(fun) & 0xFF000000) >> 24;
		res.data[2] = (get_rotate_speed(fun) & 0x00FF0000) >> 16;
		res.data[3] = (get_rotate_speed(fun) & 0x0000FF00) >> 8;
		res.data[4] = (get_rotate_speed(fun) & 0x000000FF) >> 0;
		break;
	case CMD_STEPMOTOR:
		//到达限位，且为正转
		if(SM_Limit() && (m->data[0] == CounterClockWise)){
			warning |= MOTOR_LIMIT;
		}else
			warning &= ~MOTOR_LIMIT;
		
		//驱动芯片出错
		if(SM_Fault())
			warning |= MOTOR_FAULT;
		else
			warning &= ~MOTOR_FAULT;

		if((warning & MOTOR_LIMIT) || (warning & MOTOR_FAULT))
			break;
		
		if(rotate != m->data[0]){
			rotate = m->data[0]? ClockWise: CounterClockWise;
			beats = ((m->data[1] & 0x00FF) << 8) + m->data[2];
			beatCount = 0;
			BeatCycle = BEAT_TICKS;
		}else{
			beats += ((m->data[1] & 0x00FF) << 8) + m->data[2];
			if(!BeatCycle)
				BeatCycle = BEAT_TICKS;
		}
		break;
		
	case CMD_SLEEP:
		if(m->data[0])
			SM_Sleep();
		else
			SM_Wake();
		break;
		
	case CMD_GET_TEMPE_VALUE:
		//if(m->data[0] == 0x01)
		{
			double tempe_env = get_temperauter_value(TEMP_SENSOR_0);
			res.data[0] = (uint8_t)tempe_env;
			res.data[1] = (uint8_t)((tempe_env - res.data[0])*100);
			
			double tempe_ledr = get_temperauter_value(TEMP_SENSOR_1);
			res.data[2] = (uint8_t)tempe_ledr;
			res.data[3] = (uint8_t)((tempe_ledr - res.data[0])*100); 
		}
		break;

	case CMD_GET_LIGHT_INTENSITY:
		{
			double li0 = get_light_value(LIGHT_SENSOR_0);
			double li1 = get_light_value(LIGHT_SENSOR_1);
			res.data[0] = ((uint16_t)li0 & 0xFF00)>>8;
			res.data[1] = (uint16_t)li0 & 0x00FF;
			res.data[2] = ((uint16_t)li1 & 0xFF00)>>8;
			res.data[3] = (uint16_t)li1 & 0x00FF;
		}
		break;

	case CMD_GET_ENV:
		FLASH_If_GetEnv(APPLICATION_ADDRESS, res.data, BUFFERSIZE - 4);
		break;

	case CMD_SET_ENV:
	{
		uint32_t destination = APPLICATION_ADDRESS;
		uint32_t env= 0x00;
		uint8_t i;

		for(i = 0; i < BUFFERSIZE - 4; i++){
			env <<= 8;
			env += m->data[BUFFERSIZE - 5 - i];
		}

		if((!FLASH_If_Erase(destination))
			&&(!FLASH_If_Write(&destination, &env, 1))
			&&(m->data[0] == KEY_UPGRADE))
			//NVIC_SystemReset();
		break;
	}

	case CMD_LIMIT:
	{
		//上报告警信息
		res.data[0] = warning;
		warning = 0;
		//解除gpio告警状态
	}
		break;

	case CMD_XFM:
	{
		uint16_t addr = 0, value = 0;
		
		addr += m->data[1];
		addr <<= 8;
		addr += m->data[2];

		value += m->data[3];
		value <<= 8;
		value += m->data[4];

		if(m->data[0])
			xfm_ReadReg(addr,&value);
		else
			xfm_WriteReg(addr, value);
		break;
	}
	default:
		break;
	}
	res.check = sum((uint8_t*)&res, BUFFERSIZE - 1);
//transfer:
	USART_Transfer((uint8_t*)&res, BUFFERSIZE);
}

/*
 * @brief  数据求和
 * @param  
 * @retval 
 */

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
