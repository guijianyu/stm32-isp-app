/**
  ******************************************************************************
  * @file    stm32f0xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    23-March-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
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
#include "stm32f0xx_it.h"
#include <stdio.h>
#include "string.h"
#include "led.h"
#include "uart.h"
#include "stepmotor.h"
#include "fun.h"
#include "gpio.h"
#include "bsp_key.h"

extern __IO uint32_t TimeOut;
extern __IO uint32_t funtime;
extern __IO uint32_t BeatCycle;

__IO uint32_t IC2ReadValue1[2] = {0}, IC2ReadValue2[2] = {0};
__IO uint16_t CaptureNumber[2] = {0};
__IO uint32_t Capture[2] = {0};
extern __IO uint32_t TIM2CHFreq[];

/** @addtogroup STM32F0-Discovery_Demo
  * @{
  */

/** @addtogroup STM32F0XX_IT
  * @brief Interrupts driver modules
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
	printf("NMI_Handler ... \r\n");
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
	printf("HardFault_Handler ... \r\n");
	/* Go to infinite loop when Hard Fault exception occurs */
	while (1)
	{
	}
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
	printf("SVC_Handler ... \r\n");
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
	printf("PendSV_Handler ... \r\n");
}

extern volatile uint32_t g_uiDelayCount;
/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	static uint8_t s_count = 0;
	/* Decrement the timeout value */
	if (TimeOut != 0x00){
		TimeOut--;
	}

	/*风扇延迟计数*/
	fun_tick();
	
	/* 每隔1ms进来1次 （仅用于 bsp_DelayMS） */
	if (g_uiDelayCount != 0)
	{
		g_uiDelayCount--;
	}
	
	/*步进电机，节拍控制*/
	if(BeatCycle != 0x00){
		BeatCycle--;
		if(!BeatCycle)
			beat();
	}
	
	if (++s_count >= 10)  //中断每1ms进入一次，故使用10ms需要10次
	{
		s_count = 0;

		bsp_key_scan();	/* 每隔10ms调用一次此函数，此函数在gpio.c*/
	}
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET){
		USART_Receive(USART2, USART_ReceiveData(USART2));
	}
}

void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET){
		USART_Receive(USART1, USART_ReceiveData(USART1));
	}
}

void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2, TIM_IT_CC3) == SET) 
	{
		if(TIM_GetFlagStatus(TIM2,TIM_FLAG_CC3OF)) /* Check the overflow */
		{
			/* Overflow error management */
			CaptureNumber[0] = 0; 	/* Reinitialize the laps computing */
			TIM_ClearFlag(TIM2, TIM_FLAG_CC3 | TIM_FLAG_CC3OF); /* Clear the flags */
			return;
		}			
		/* Clear TIM2 Capture compare interrupt pending bit */
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);
		if(CaptureNumber[0] == 0){
			/* Get the Input Capture value */
			IC2ReadValue1[0] = TIM_GetCapture3(TIM2);
			CaptureNumber[0] = 1;

		}else if(CaptureNumber[0] == 1){
			/* Get the Input Capture value */
			IC2ReadValue2[0] = TIM_GetCapture3(TIM2); 
			
			/* Capture computation */
			if (IC2ReadValue2[0] > IC2ReadValue1[0]){
				Capture[0] = (IC2ReadValue2[0] - IC2ReadValue1[0]); 
			}else if (IC2ReadValue2[0] < IC2ReadValue1[0]){
				Capture[0] = ((0xFFFF - IC2ReadValue1[0]) + IC2ReadValue2[0]); 
			}else{
				Capture[0] = 0;
			}

			if(Capture[0] != 0)
			{
				/* Frequency computation */ 
				TIM2CHFreq[0] = (uint32_t) SystemCoreClock / Capture[0] / TIM2_PRESCALER;
			}
			else 
			{
				TIM2CHFreq[0] = 0;
			}
			CaptureNumber[0] = 0;
		}
	}

	if(TIM_GetITStatus(TIM2, TIM_IT_CC4) == SET) {
		if(TIM_GetFlagStatus(TIM2,TIM_FLAG_CC4OF)) /* Check the overflow */
		{
			/* Overflow error management */
			CaptureNumber[1] = 0; 	/* Reinitialize the laps computing */
			TIM_ClearFlag(TIM2, TIM_FLAG_CC4 | TIM_FLAG_CC4OF); /* Clear the flags */
			return;
		}			
		/* Clear TIM2 Capture compare interrupt pending bit */
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
		
		if(CaptureNumber[1] == 0){
			/* Get the Input Capture value */
			IC2ReadValue1[1] = TIM_GetCapture4(TIM2);
			CaptureNumber[1] = 1;
		
		}else if(CaptureNumber[1] == 1){
			/* Get the Input Capture value */
			IC2ReadValue2[1] = TIM_GetCapture4(TIM2); 
			
			/* Capture computation */
			if (IC2ReadValue2[1] > IC2ReadValue1[1]){
				Capture[1] = (IC2ReadValue2[1] - IC2ReadValue1[1]); 
			}else if (IC2ReadValue2[1] < IC2ReadValue1[1]){
				Capture[1] = ((0xFFFF - IC2ReadValue1[1]) + IC2ReadValue2[1]); 
			}else{
				Capture[1] = 0;
			}

			if(Capture[1] != 0)
			{
				/* Frequency computation */ 
				TIM2CHFreq[1] = (uint32_t) SystemCoreClock / Capture[1] / TIM2_PRESCALER;
			}
			else
			{
				TIM2CHFreq[1] = 0;
			}
			CaptureNumber[1] = 0;
		}
	}

}

/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f0xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
