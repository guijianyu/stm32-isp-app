#include "dlp_control.h"
uint16_t g_dlp_out_pwm_TIM_Period = 0;
__IO uint32_t g_DlpFrequency = 0;

/*
*********************************************************************************************************
*	�� �� ��: bsp_GetRCCofGPIO
*	����˵��: ����GPIO�˿ڵõ���Ӧ��GPIO RCC�Ĵ���
*	��    �Σ���
*	�� �� ֵ: GPIO����ʱ����
*********************************************************************************************************
*/
uint32_t bsp_GetRCCofGPIO(GPIO_TypeDef* GPIOx)
{
	uint32_t rcc;

	if (GPIOx == GPIOA)
	{
		rcc = RCC_AHBPeriph_GPIOA;
	}
	else if (GPIOx == GPIOB)
	{
		rcc = RCC_AHBPeriph_GPIOB;
	}
	else if (GPIOx == GPIOC)
	{
		rcc = RCC_AHBPeriph_GPIOC;
	}
	else if (GPIOx == GPIOD)
	{
		rcc = RCC_AHBPeriph_GPIOD;
	}
	else if (GPIOx == GPIOF)
	{
		rcc = RCC_AHBPeriph_GPIOF;
	}
	
	return rcc;
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_GetPinSource
*	����˵��: ���� GPIO_Pin_X �õ� GPIO_PinSource
*	��    �Σ�gpio_pin
*	�� �� ֵ: GPIO_PinSource
*********************************************************************************************************
*/
uint8_t bsp_GetPinSource(uint16_t gpio_pin)
{
	uint8_t ret;

	if (gpio_pin == GPIO_Pin_0)
	{
		ret = GPIO_PinSource0;
	}
	else if (gpio_pin == GPIO_Pin_1)
	{
		ret = GPIO_PinSource1;
	}
	else if (gpio_pin == GPIO_Pin_2)
	{
		ret = GPIO_PinSource2;
	}
	else if (gpio_pin == GPIO_Pin_3)
	{
		ret = GPIO_PinSource3;
	}
	else if (gpio_pin == GPIO_Pin_4)
	{
		ret = GPIO_PinSource4;
	}
	else if (gpio_pin == GPIO_Pin_5)
	{
		ret = GPIO_PinSource5;
	}
	else if (gpio_pin == GPIO_Pin_6)
	{
		ret = GPIO_PinSource6;
	}
	else if (gpio_pin == GPIO_Pin_7)
	{
		ret = GPIO_PinSource7;
	}
	else if (gpio_pin == GPIO_Pin_8)
	{
		ret = GPIO_PinSource8;
	}
	else if (gpio_pin == GPIO_Pin_9)
	{
		ret = GPIO_PinSource9;
	}
	else if (gpio_pin == GPIO_Pin_10)
	{
		ret = GPIO_PinSource10;
	}
	else if (gpio_pin == GPIO_Pin_11)
	{
		ret = GPIO_PinSource11;
	}
	else if (gpio_pin == GPIO_Pin_12)
	{
		ret = GPIO_PinSource12;
	}
	else if (gpio_pin == GPIO_Pin_13)
	{
		ret = GPIO_PinSource13;
	}
	else if (gpio_pin == GPIO_Pin_14)
	{
		ret = GPIO_PinSource14;
	}
	else if (gpio_pin == GPIO_Pin_15)
	{
		ret = GPIO_PinSource15;
	}

	return ret;
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_GetRCCofTIM
*	����˵��: ����TIMx �õ�timer��RCC�Ĵ���
*	��    �Σ���
*	�� �� ֵ: TIM����ʱ����
*********************************************************************************************************
*/
uint32_t bsp_GetRCCofTIM(TIM_TypeDef* TIMx)
{
	uint32_t rcc;

	/*
		APB1 ��ʱ���� TIM2, TIM3 ,TIM6, TIM14
		APB2 ��ʱ���� TIM1, TIM15 ,TIM16, TIM17
	*/
	
	/* ������ APB2ʱ�� */
	if (TIMx == TIM1)
	{
		rcc = RCC_APB2Periph_TIM1;
	}
	else if (TIMx == TIM15)
	{
		rcc = RCC_APB2Periph_TIM15;
	}
	else if (TIMx == TIM16)
	{
		rcc = RCC_APB2Periph_TIM16;
	}
	else if (TIMx == TIM17)
	{
		rcc = RCC_APB2Periph_TIM17;
	}

	/* ������ APB1ʱ�� */
	else if (TIMx == TIM2)
	{
		rcc = RCC_APB1Periph_TIM2;
	}
	else if (TIMx == TIM3)
	{
		rcc = RCC_APB1Periph_TIM3;
	}
	else if (TIMx == TIM6)
	{
		rcc = RCC_APB1Periph_TIM6;
	}
	else if (TIMx == TIM14)
	{
		rcc = RCC_APB1Periph_TIM14;
	}

	return rcc;
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_ConfigTimGpio
*	����˵��: ����timer��ص�GPIO
*	��    ��: GPIOx
*			 GPIO_PinX
*			 TIMx
*			 _ucChannel
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_ConfigTimGpio(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pinx, uint8_t GPIO_AF_x)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	/* ʹ��GPIOʱ�� */
	RCC_AHBPeriphClockCmd(bsp_GetRCCofGPIO(GPIOx), ENABLE);

	/* ����GPIO */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pinx;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOx, &GPIO_InitStructure);

	/* ���ӵ�AF���� */
	GPIO_PinAFConfig(GPIOx, bsp_GetPinSource(GPIO_Pinx), GPIO_AF_x);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_ConfigGpioOut
*	����˵��: ����GPIOΪ�����������Ҫ����PWM�����ռ�ձ�Ϊ0��100�������
*	��    ��: GPIOx
*			  GPIO_PinX
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_ConfigGpioOut(GPIO_TypeDef* GPIOx, uint16_t GPIO_PinX)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	/* ʹ��GPIOʱ�� */
	RCC_AHBPeriphClockCmd(bsp_GetRCCofGPIO(GPIOx), ENABLE);

	/* ����GPIO */
	GPIO_InitStructure.GPIO_Pin = GPIO_PinX;		/* ������β� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	/* ��� */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	/* ���� */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;	/* ������ */
	GPIO_Init(GPIOx, &GPIO_InitStructure);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_SetTIMOutPWM
*	����˵��: �������������PWM�źŵ�Ƶ�ʺ�ռ�ձ�
*	��    ��: _ulFreq : PWM�ź�Ƶ�ʣ���λHz. 0 ��ʾ��ֹ���
*			  _ulDutyCycle : PWM�ź�ռ�ձȣ���λ�����֮һ����5000����ʾ50.00%��ռ�ձ�
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_SetTIMOutPWM(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t GPIO_AF_x, TIM_TypeDef* TIMx, uint8_t _ucChannel,
	 uint32_t _ulFreq, uint32_t _ulDutyCycle)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	uint16_t usPeriod;		/* �Զ���װ��ֵ */
	uint16_t usPrescaler;	/* ��Ƶ�� */
	uint32_t uiTIMxCLK;
	uint16_t ChannelxPulse; /*ռ�ձȼ���ֵ����ָ��������ֵ��װ�뵽��׽�ȽϼĴ���*/
 
	if (_ulDutyCycle == 0)
	{		
		TIM_Cmd(TIMx, DISABLE);		/* �ر�PWM��� */
		bsp_ConfigGpioOut(GPIOx, GPIO_Pin);	/* ����GPIOΪ������� */		
		GPIO_WriteBit(GPIOx, GPIO_Pin, Bit_RESET);	/* PWM = 0 */		
		return;
	}
	else if (_ulDutyCycle == 10000)
	{
		TIM_Cmd(TIMx, DISABLE);		/* �ر�PWM��� */
		bsp_ConfigGpioOut(GPIOx, GPIO_Pin);	/* ����GPIOΪ������� */		
		GPIO_WriteBit(GPIOx, GPIO_Pin, Bit_SET);	/* PWM = 1 */	
		return;
	}	
	
	bsp_ConfigTimGpio(GPIOx, GPIO_Pin, GPIO_AF_x);	/* ����GPIO */

	/*
		APB1 ��ʱ���� TIM2, TIM3 ,TIM6, TIM14
		APB2 ��ʱ���� TIM1, TIM15 ,TIM16, TIM17
	*/
	
  	/*TIMx clock enable*/
	if ((TIMx == TIM1) || (TIMx == TIM15) || (TIMx == TIM16) || (TIMx == TIM17))
	{
		RCC_APB2PeriphClockCmd(bsp_GetRCCofTIM(TIMx), ENABLE);
	}
	else
	{
		RCC_APB1PeriphClockCmd(bsp_GetRCCofTIM(TIMx), ENABLE);
	}	
	
    /*-----------------------------------------------------------------------
		system_stm32f4xx.c �ļ��� void SetSysClock(void) ������ʱ�ӵ��������£�
		HCLK = SYSCLK
		PCLK = HCLK 
	
		HCLK = SYSCLK / 1     (AHB1Periph)
		PCLK2 = HCLK / 1      (APB2Periph)
		PCLK1 = HCLK / 1      (APB1Periph)
	
		TIM1CLK = PCLK2 = SystemCoreClock = 48 MHz
		TIM3CLK = PCLK1 = SystemCoreClock = 48 MHz
		APB1 ��ʱ���� TIM2, TIM3 ,TIM6, TIM14
		APB2 ��ʱ���� TIM1, TIM15 ,TIM16, TIM17
	----------------------------------------------------------------------- */
	if ((TIMx == TIM1) || (TIMx == TIM15) || (TIMx == TIM16) || (TIMx == TIM17)) /* APB2 ��ʱ�� */
	{		
		uiTIMxCLK = SystemCoreClock;
	}
	else	/* APB1 ��ʱ�� */
	{
		uiTIMxCLK = SystemCoreClock;
	}

	if (_ulFreq < 100)
	{
		usPrescaler = 10000 - 1;							/* ��Ƶ�� = 10000 */
		usPeriod =  (uiTIMxCLK / 10000) / _ulFreq  - 1;		/* �Զ���װ��ֵ */		
	}	
	else if (_ulFreq < 3000)
	{
		usPrescaler = 100 - 1;								/* ��Ƶ�� = 100 */
		usPeriod =  (uiTIMxCLK / 100) / _ulFreq  - 1;		/* �Զ���װ��ֵ */
	}
	else	/* ����4K��Ƶ�ʣ������Ƶ */
	{
		usPrescaler = 0;									/* ��Ƶ�� = 1 */
		usPeriod = uiTIMxCLK / _ulFreq - 1;					/* �Զ���װ��ֵ */
	}
	g_dlp_out_pwm_TIM_Period = usPeriod;
	ChannelxPulse = (uint16_t) (((uint32_t) _ulDutyCycle * (usPeriod - 1)) / 10000);
	
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Prescaler = usPrescaler;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	
	TIM_TimeBaseStructure.TIM_Period = usPeriod;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;	

	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);

	/*  Channelx Configuration in PWM mode  */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  /*���ϼ���ģʽʱ��PWM1Ϊ�ȸߵ�ƽ�ٵ͵�ƽ��PWM2Ϊ�ȵڵ͵�ƽ�ٸߵ�ƽ*/
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = ChannelxPulse;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;	

	if (_ucChannel == TIM_Channel_1)
	{
		TIM_OC1Init(TIMx, &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(TIMx, TIM_OCPreload_Enable);
	}
	else if (_ucChannel == TIM_Channel_2)
	{
		TIM_OC2Init(TIMx, &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(TIMx, TIM_OCPreload_Enable);
	}
	else if (_ucChannel == TIM_Channel_3)
	{
		TIM_OC3Init(TIMx, &TIM_OCInitStructure);
		TIM_OC3PreloadConfig(TIMx, TIM_OCPreload_Enable);
	}
	else if (_ucChannel == TIM_Channel_4)
	{
		TIM_OC4Init(TIMx, &TIM_OCInitStructure);
		TIM_OC4PreloadConfig(TIMx, TIM_OCPreload_Enable);
	}

	TIM_ARRPreloadConfig(TIMx, ENABLE);

	/* TIMx enable counter */
	TIM_Cmd(TIMx, ENABLE);

	/* ������仰����TIM1 15 16 17�Ǳ���ģ�����TIM2-TIM6�򲻱�Ҫ */
	if ((TIMx == TIM1) || (TIMx == TIM15) || (TIMx == TIM16) || (TIMx == TIM17))
	{
		/* TIMx Main Output Enable */
		TIM_CtrlPWMOutputs(TIMx, ENABLE);
	}
}


/*Select the TIMx PWM Input Trigger*/
uint16_t bsp_SelectPWMTrigger(uint8_t _ucChannel, uint16_t TIM_ICSelection)
{
	uint16_t input_trigger = TIM_TS_TI1FP1;
	if(_ucChannel == TIM_Channel_1)
	{
		if(TIM_ICSelection_DirectTI == TIM_ICSelection)
		{
			input_trigger = TIM_TS_TI1FP1; 
		}
		else
		{
			input_trigger = TIM_TS_TI2FP2; 
		}
	}
	else if (_ucChannel == TIM_Channel_2)
	{
		if(TIM_ICSelection_DirectTI == TIM_ICSelection)
		{
			input_trigger = TIM_TS_TI2FP2; 
		}
		else
		{
			input_trigger = TIM_TS_TI1FP1; 
		}
	}
	return input_trigger;	
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_SetTIMInPWM
*	����˵��: �������������PWM�źŵ�Ƶ�ʺ�ռ�ձ�
*	��    ��: _ulFreq : PWM�ź�Ƶ�ʣ���λHz. 0 ��ʾ��ֹ���
*			  _ulDutyCycle : PWM�ź�ռ�ձȣ���λ�����֮һ����5000����ʾ50.00%��ռ�ձ�
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_SetTIMInPWM(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t GPIO_AF_x, TIM_TypeDef* TIMx, uint8_t _ucChannel, uint8_t TIMx_IRQn,
	uint16_t TIMx_IT_Event)
{
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	uint16_t usPeriod;		/* �Զ���װ��ֵ */
	uint16_t usPrescaler;	/* ��Ƶ�� */
	uint32_t uiTIMxCLK;
 
	bsp_ConfigTimGpio(GPIOx, GPIO_Pin, GPIO_AF_x);	/* ����GPIO */

	/*
		APB1 ��ʱ���� TIM2, TIM3 ,TIM6, TIM14
		APB2 ��ʱ���� TIM1, TIM15 ,TIM16, TIM17
	*/
	
  	/*TIMx clock enable*/
	if ((TIMx == TIM1) || (TIMx == TIM15) || (TIMx == TIM16) || (TIMx == TIM17))
	{
		RCC_APB2PeriphClockCmd(bsp_GetRCCofTIM(TIMx), ENABLE);
	}
	else
	{
		RCC_APB1PeriphClockCmd(bsp_GetRCCofTIM(TIMx), ENABLE);
	}	
	
	/* exeample:
		TIM2 configuration: PWM Input mode
		 The external signal is connected to TIM2 CH2 pin (PA.01)
		 TIM2 CCR2 is used to compute the frequency value 
		 TIM2 CCR1 is used to compute the duty cycle value

		In this example TIM2 input clock (TIM2CLK) is set to APB1 clock (PCLK1), since
		APB1 prescaler is set to 1.
		  TIM2CLK = PCLK1 = HCLK = SystemCoreClock

		External Signal Frequency = SystemCoreClock / TIM2_CCR2 in Hz.
		External Signal DutyCycle = (TIM2_CCR1*100)/(TIM2_CCR2) in %.	
	*/
 
	TIM_ICInitStructure.TIM_Channel = _ucChannel;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;

	/*TIM1 2 3 15 ֧��PWM���벶��ģʽ*/
	if ((TIMx == TIM1) || (TIMx == TIM2) || (TIMx == TIM3) || (TIMx == TIM15))
	{
		TIM_PWMIConfig(TIMx, &TIM_ICInitStructure);
		
		/* Select the TIMx Input Trigger: TI2FP2 */
		TIM_SelectInputTrigger(TIMx, bsp_SelectPWMTrigger(_ucChannel,TIM_ICInitStructure.TIM_ICPolarity));

		/* Select the slave Mode: Reset Mode */
		TIM_SelectSlaveMode(TIMx, TIM_SlaveMode_Reset);
		TIM_SelectMasterSlaveMode(TIMx,TIM_MasterSlaveMode_Enable);

		/* TIM enable counter */
		TIM_Cmd(TIMx, ENABLE);

		/* Enable the CCx Interrupt Request */
		TIM_ITConfig(TIMx, TIMx_IT_Event, ENABLE);   		
	}
	else 
	{
		uiTIMxCLK = SystemCoreClock;
		usPrescaler = (uint16_t) (uiTIMxCLK  / DLP_ICPWM_SAMPLING_CLK) - 1;
		usPeriod = 0xFFFF - 1;
				
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		/* Time base configuration */
		TIM_TimeBaseStructure.TIM_Prescaler = usPrescaler;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	
		TIM_TimeBaseStructure.TIM_Period = usPeriod;
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;	
		TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);		
		
		/*TIM Input caputure config*/  
		//TIM_ICInitStructure.TIM_ICFilter = 0x1;	//N=2	�˲�����������2��
		TIM_ICInit(TIMx, &TIM_ICInitStructure);
	
		/* TIM enable counter */
		TIM_Cmd(TIMx, ENABLE);
		
		/* Enable the CCx Interrupt Request */
		TIM_ITConfig(TIMx, TIMx_IT_Event, ENABLE);	
	}
	
	/* Enable the TIMx global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIMx_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
}

void DLP_TIM_IRQHandler(void)
{
	static __IO uint16_t ICxValue1 = 0;
	static __IO uint16_t ICxValue2 = 0;
	static __IO uint32_t CaptureCount = 0;
	__IO uint16_t ICxValue = 0;

	if(TIM_GetITStatus(DLP_ICPWM_TIM, DLP_TIM_IT_Event) == SET)
	{	
		 /* Check the overflow */
		switch(DLP_TIM_IT_Event)
		{
			case TIM_IT_CC1:
				if(TIM_GetFlagStatus(DLP_ICPWM_TIM,TIM_FLAG_CC1OF)) /* Check the overflow */
				{
					/* Overflow error management */
					CaptureCount = 0; 	/* Reinitialize the laps computing */
					TIM_ClearFlag(DLP_ICPWM_TIM, TIM_FLAG_CC1 | TIM_FLAG_CC1OF); /* Clear the flags */
					return;
				}	
				break;
			case TIM_IT_CC2:
				if(TIM_GetFlagStatus(DLP_ICPWM_TIM,TIM_FLAG_CC2OF)) /* Check the overflow */
				{
					/* Overflow error management */
					CaptureCount = 0; 	/* Reinitialize the laps computing */
					TIM_ClearFlag(DLP_ICPWM_TIM, TIM_FLAG_CC2 | TIM_FLAG_CC2OF); /* Clear the flags */
					return;
				}				
				break;
			case TIM_IT_CC3:
				if(TIM_GetFlagStatus(DLP_ICPWM_TIM,TIM_FLAG_CC3OF)) /* Check the overflow */
				{
					/* Overflow error management */
					CaptureCount = 0; 	/* Reinitialize the laps computing */
					TIM_ClearFlag(DLP_ICPWM_TIM, TIM_FLAG_CC3| TIM_FLAG_CC3OF); /* Clear the flags */
					return;
				}				
				break;
			case TIM_IT_CC4:
				if(TIM_GetFlagStatus(DLP_ICPWM_TIM,TIM_FLAG_CC4OF)) /* Check the overflow */
				{
					/* Overflow error management */
					CaptureCount = 0; 	/* Reinitialize the laps computing */
					TIM_ClearFlag(DLP_ICPWM_TIM, TIM_FLAG_CC4 | TIM_FLAG_CC4OF); /* Clear the flags */
					return;
				}	
				break;
			default :
				break;		
		}
	
		/* Clear TIMx Capture compare interrupt pending bit */
		TIM_ClearITPendingBit(DLP_ICPWM_TIM, DLP_TIM_IT_Event);

		if(CaptureCount == 0)
		{
			switch(DLP_TIM_IT_Event)
			{
				case TIM_IT_CC1:				
					ICxValue1 = TIM_GetCapture1(DLP_ICPWM_TIM);
					break;
				case TIM_IT_CC2:
					ICxValue1 = TIM_GetCapture2(DLP_ICPWM_TIM);				
					break;
				case TIM_IT_CC3:
					ICxValue1 = TIM_GetCapture3(DLP_ICPWM_TIM);				
					break;
				case TIM_IT_CC4:
					ICxValue1 = TIM_GetCapture4(DLP_ICPWM_TIM);
					break;
				default :
					break;
			}
			CaptureCount = 1;
		}
		else 
		{
			switch(DLP_TIM_IT_Event)
			{
				case TIM_IT_CC1:
					ICxValue2 = TIM_GetCapture1(DLP_ICPWM_TIM);
					break;
				case TIM_IT_CC2:
					ICxValue2 = TIM_GetCapture2(DLP_ICPWM_TIM); 			
					break;
				case TIM_IT_CC3:
					ICxValue2 = TIM_GetCapture3(DLP_ICPWM_TIM); 			
					break;
				case TIM_IT_CC4:
					ICxValue2 = TIM_GetCapture4(DLP_ICPWM_TIM);
					break;
				default :
					break;				
			}
			/* Capture computation */
			if (ICxValue2 > ICxValue1){
				ICxValue = ICxValue2 - ICxValue1; 
			}else {
				ICxValue = 0xFFFF - ICxValue1 + ICxValue2 + 1; 
			}
	
			if(ICxValue != 0)
			{
				g_DlpFrequency = SystemCoreClock / ICxValue / (SystemCoreClock  / DLP_ICPWM_SAMPLING_CLK);
			}
			else
			{
				g_DlpFrequency = 0;
			}	
			
			CaptureCount = 0;
			ICxValue1 = ICxValue2;
		}
	}
	else
	{

	}
}

/*
*********************************************************************************************************
*	�� �� ��: OutPwmToDLP_Init
*	����˵��: stm32оƬ���pwm��DLPоƬ�ĳ�ʼ��
*	��    ��: ��
*			  
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void OutPwmToDLP_Init(void)
{
	bsp_SetTIMOutPWM(DLP_GREEN_LIGHT_PWM_PORT, DLP_GREEN_LIGHT_PWM_PIN, DLP_GREEN_LIGHT_PWM_AF, 
					 DLP_GREEN_LIGHT_PWM_TIM, DLP_GREEN_LIGHT_PWM_TIM_CHANNEL, DLP_GREEN_LIGHT_PWM_FREQ, DLP_GREEN_LIGHT_PWM_DUTY_CYCLE);	

	//bsp_SetTIMOutPWM(DLP_SDIM_PWM_PORT, DLP_SDIM_PWM_PIN, DLP_SDIM_PWM_AF, 
	//				 DLP_SDIM_PWM_TIM, DLP_SDIM_PWM_TIM_CHANNEL, DLP_SDIM_PWM_FREQ, DLP_SDIM_PWM_DUTY_CYCLE);	
}

/*
*********************************************************************************************************
*	�� �� ��: InPwmFromDLP_Init
*	����˵��: DLPоƬ���PWM��stm32оƬ
*	��    ��: ��
*			  
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void InPwmFromDLP_Init(void)
{
	bsp_SetTIMInPWM(DLP_ICPWM_GPIO_PORT, DLP_ICPWM_PIN, DLP_ICPWM_AF, DLP_ICPWM_TIM, DLP_ICPWM_CHANNEL, DLP_TIM_IRQn, DLP_TIM_IT_Event);
}

/*
*********************************************************************************************************
*	�� �� ��: DLP_Init
*	����˵��: DLPоƬ���PWM��stm32оƬ
*	��    ��: ��
*			  
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void DLP_Init(void)
{
	OutPwmToDLP_Init();
	//InPwmFromDLP_Init();
}

/*
*********************************************************************************************************
*	�� �� ��: DLP_Set_Pulse
*	����˵��: �������������PWM�źŵ�Ƶ�ʺ�ռ�ձ�
*	��    ��:  usPeriod : TIM��Ԥװ����ֵ������ֵ
*			  _ulDutyCycle : PWM�ź�ռ�ձȣ���λ�����֮һ����5000����ʾ50.00%��ռ�ձ�
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void DLP_Set_Pulse(TIM_TypeDef* TIMx, uint8_t _ucChannel,  uint16_t usPeriod, uint32_t _ulDutyCycle)
{
	uint16_t ChannelxPulse; /*ռ�ձȼ���ֵ����ָ��������ֵ��װ�뵽��׽�ȽϼĴ���*/
	static uint8_t pwm_flag = 0;  //1����ʾ��ǰpwmΪ0����100��  0����ʾʹ��TIM���pwm
	if(_ulDutyCycle > 10000)
	{
		_ulDutyCycle = 10000;
	}
	if (_ulDutyCycle == 0)
	{		
		pwm_flag = 1;
		TIM_Cmd(TIMx, DISABLE);		/* �ر�PWM��� */
		bsp_ConfigGpioOut(DLP_GREEN_LIGHT_PWM_PORT, DLP_GREEN_LIGHT_PWM_PIN);	/* ����GPIOΪ������� */		
		GPIO_WriteBit(DLP_GREEN_LIGHT_PWM_PORT, DLP_GREEN_LIGHT_PWM_PIN, Bit_RESET);	/* PWM = 0 */		
		return;
	}
	else if (_ulDutyCycle == 10000)
	{
		pwm_flag = 1; 
		TIM_Cmd(TIMx, DISABLE);		/* �ر�PWM��� */
		bsp_ConfigGpioOut(DLP_GREEN_LIGHT_PWM_PORT, DLP_GREEN_LIGHT_PWM_PIN);	/* ����GPIOΪ������� */		
		GPIO_WriteBit(DLP_GREEN_LIGHT_PWM_PORT, DLP_GREEN_LIGHT_PWM_PIN, Bit_SET);	/* PWM = 1 */	
		return;
	}	
	else 
	{	
		if(1 == pwm_flag)
		{
			bsp_SetTIMOutPWM(DLP_GREEN_LIGHT_PWM_PORT, DLP_GREEN_LIGHT_PWM_PIN, DLP_GREEN_LIGHT_PWM_AF, 
				 DLP_GREEN_LIGHT_PWM_TIM, DLP_GREEN_LIGHT_PWM_TIM_CHANNEL, DLP_GREEN_LIGHT_PWM_FREQ, _ulDutyCycle);	
			pwm_flag = 0;  //�����־
			return ;
		}
		else
		{
			ChannelxPulse = (uint16_t) (((uint32_t) _ulDutyCycle * (usPeriod - 1)) / 10000);	
			
			if (_ucChannel == TIM_Channel_1)
			{
				TIM_SetCompare1(TIMx, ChannelxPulse);
			}
			else if (_ucChannel == TIM_Channel_2)
			{
				TIM_SetCompare2(TIMx, ChannelxPulse);
			}
			else if (_ucChannel == TIM_Channel_3)
			{
				TIM_SetCompare3(TIMx, ChannelxPulse);
			}
			else if (_ucChannel == TIM_Channel_4)
			{
				TIM_SetCompare4(TIMx, ChannelxPulse);
			}	
		}
	}
}
