#include "IWDG_model.h"

//#define  LSI_TIM_MEASURE
__IO uint32_t LsiFreq = 40000;

#ifdef LSI_TIM_MEASURE
__IO static uint16_t IC1ReadValue1 = 0, IC1ReadValue2 = 0;
__IO static uint16_t CaptureNumber = 0;

/**
  * @brief  Configures TIM14 to measure the LSI oscillator frequency.
  * @param  None
  * @retval None
  */
static void TIM14_ConfigForLSI(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;

	/* Enable peripheral clocks ------------------------------------------------*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

	/* Allow access to the RTC */
	PWR_BackupAccessCmd(ENABLE);

	/* Reset RTC Domain */
	RCC_BackupResetCmd(ENABLE);
	RCC_BackupResetCmd(DISABLE);

	/*!< LSI Enable */
	RCC_LSICmd(ENABLE);

	/*!< Wait till LSI is ready */
	while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
	{}

	/* Select the RTC Clock Source */
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);

	/* Enable the RTC Clock */
	RCC_RTCCLKCmd(ENABLE);

	/* Wait for RTC APB registers synchronisation */
	RTC_WaitForSynchro();

	/* Enable TIM14 clocks */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);  

	/* Enable the TIM14 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM14_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Configure TIM14 prescaler */
	TIM_PrescalerConfig(TIM14, 0, TIM_PSCReloadMode_Immediate);

	/* Connect internally the TM14_CH1 Input Capture to the LSI clock output */
	TIM_RemapConfig(TIM14, TIM14_RTC_CLK);

	/* TIM14 configuration: Input Capture mode ---------------------
	 The LSI oscillator is connected to TIM14 CH1
	 The Rising edge is used as active edge,
	 The TIM14 CCR1 is used to compute the frequency value 
	------------------------------------------------------------ */
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8;
	TIM_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM14, &TIM_ICInitStructure);

	/* TIM14 Counter Enable */
	TIM_Cmd(TIM14, ENABLE);

	/* Reset the flags */
	TIM14->SR = 0;

	/* Enable the CC1 Interrupt Request */  
	TIM_ITConfig(TIM14, TIM_IT_CC1, ENABLE);  
}
#endif /* LSI_TIM_MEASURE */


#ifdef LSI_TIM_MEASURE
/**
  * @brief  This function handles TIM14 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM14_IRQHandler(void)
{
	__IO uint32_t Capture = 0;	
	if (TIM_GetITStatus(TIM14, TIM_IT_CC1) != RESET)
	{    
		if(CaptureNumber == 0)
		{
			/* Get the Input Capture value */
			IC1ReadValue1 = TIM_GetCapture1(TIM14);
		}
		else if(CaptureNumber == 1)
		{
			/* Get the Input Capture value */
			IC1ReadValue2 = TIM_GetCapture1(TIM14); 

			/* Capture computation */
			if (IC1ReadValue2 > IC1ReadValue1)
			{
				Capture = (IC1ReadValue2 - IC1ReadValue1); 
			}
			else
			{
				Capture = ((0xFFFF - IC1ReadValue1) + IC1ReadValue2)+1; 
			}
			/* Frequency computation */ 
			LsiFreq = (uint32_t) SystemCoreClock / Capture;
			LsiFreq *= 8;
		}
		
		CaptureNumber++;

		/* Clear TIM14 Capture compare interrupt pending bit */
		TIM_ClearITPendingBit(TIM14, TIM_IT_CC1);
	}
}
#endif /* LSI_TIM_MEASURE */


/*
*********************************************************************************************************
*	函 数 名: bsp_InitIwdg
*	功能说明: 独立看门狗时间配置函数
*	形    参：IWDGTime: 0 ---- 0x0FFF
*			  独立看门狗时间设置,单位为ms,IWDGTime = 1000 大约就是一秒的
*             时间，这里没有结合TIMx测得实际LSI频率，只是为了操作方便取了
*             一个估计值超过IWDGTime不进行喂狗的话系统将会复位。
*			  LSI = 40000左右
*	返 回 值: 无		        
*********************************************************************************************************
*/
void bsp_InitIwdg(uint32_t _ulIWDGTime)
{		
	/* Check if the system has resumed from IWDG reset */
	if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET)
	{		
		/* Clear reset flags */
		RCC_ClearFlag();
	}
	else
	{

	}
	
#ifdef LSI_TIM_MEASURE
  /* TIM Configuration -------------------------------------------------------*/
  TIM14_ConfigForLSI();
  
  /* Wait until the TIM14 get 2 LSI edges */
  while(CaptureNumber != 2)
  {
  }

  /* Disable TIM14 CC1 Interrupt Request */
  TIM_ITConfig(TIM14, TIM_IT_CC1, DISABLE);
#else 
	/*!< LSI Enable */
	RCC_LSICmd(ENABLE);
	
	/*!< Wait till LSI is ready */
	while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
	{}
	
    /*set  LsiFreq*/		
	LsiFreq = 40000;  
#endif /* LSI_TIM_MEASURE */	

    /*Write 0x5555 to Enable write access to IWDG_PR and IWDG_RLR registers */
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	
	/* IWDG counter clock: LSI/32 */
	IWDG_SetPrescaler(IWDG_Prescaler_32);
	
	/*特别注意，由于这里_ulIWDGTime的最小单位是ms, 所以这里重装计数的
	  计数时 需要除以1000
	 Counter Reload Value = (_ulIWDGTime / 1000) /(1 / IWDG counter clock period)
	                      = (_ulIWDGTime / 1000) / (32/LSI)
	                      = (_ulIWDGTime / 1000) / (32/LsiFreq)
	                      = LsiFreq * _ulIWDGTime / 32000
	 实际测试LsiFreq = 34000，所以这里取1的时候 大概就是1ms 
	*/
	
	IWDG_SetReload(LsiFreq*_ulIWDGTime/32000);
	
	/* Reload IWDG counter */
	IWDG_ReloadCounter();
	
	/* Enable IWDG (the LSI oscillator will be enabled by hardware) */
	IWDG_Enable();		
}

/*
*********************************************************************************************************
*	函 数 名: IWDG_Feed
*	功能说明: 喂狗函数
*	形    参：无
*	返 回 值: 无		        
*********************************************************************************************************
*/
void IWDG_Feed(void)
{
	/* Reload IWDG counter */
	IWDG_ReloadCounter();
}
