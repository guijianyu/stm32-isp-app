#include <stdio.h>
#include "gpio.h"
#include "stm32f0xx_exti.h"
#include "dlp_control.h"
#include "xfm10213_model.h"
#include "led.h"
#include "bsp_key.h"

static __IO uint8_t sg_reset_XFM10213_flag = 0;
static __IO uint8_t sg_reset_I94113_flag = 0;
static __IO uint8_t sg_wake_648_flag = 0;
__IO uint8_t g_648_work_state = SLEEP_STATE;  //Ĭ��˯��״̬
__IO uint8_t g_wifi_wake_flag = 0;
/*
*********************************************************************************************************
*	�� �� ��: bsp_GetExitPortOfGPIO
*	����˵��: ����GPIO�˿ڵõ���Ӧ��Exit Port�Ĵ���
*	��    �Σ���
*	�� �� ֵ: �ⲿ�ж϶˿ں�
*********************************************************************************************************
*/
uint8_t bsp_GetExitPortOfGPIO(GPIO_TypeDef* GPIOx)
{
	uint32_t exit_port;

	if (GPIOx == GPIOA)
	{
		exit_port = EXTI_PortSourceGPIOA;
	}
	else if (GPIOx == GPIOB)
	{
		exit_port = EXTI_PortSourceGPIOB;
	}
	else if (GPIOx == GPIOC)
	{
		exit_port = EXTI_PortSourceGPIOC;
	}
	else if (GPIOx == GPIOD)
	{
		exit_port = EXTI_PortSourceGPIOD;
	}
	else if (GPIOx == GPIOF)
	{
		exit_port = EXTI_PortSourceGPIOF;
	}
	
	return exit_port;
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_GetExitPinSource
*	����˵��: ���� GPIO_Pin_X �õ� EXTI_PinSourcexx
*	��    �Σ�gpio_pin
*	�� �� ֵ: EXTI_PinSourcexx
*********************************************************************************************************
*/
uint8_t bsp_GetExitPinSource(uint16_t gpio_pin)
{
	uint8_t ret;

	if (gpio_pin == GPIO_Pin_0)
	{
		ret = EXTI_PinSource0;
	}
	else if (gpio_pin == GPIO_Pin_1)
	{
		ret = EXTI_PinSource1;
	}
	else if (gpio_pin == GPIO_Pin_2)
	{
		ret = EXTI_PinSource2;
	}
	else if (gpio_pin == GPIO_Pin_3)
	{
		ret = EXTI_PinSource3;
	}
	else if (gpio_pin == GPIO_Pin_4)
	{
		ret = EXTI_PinSource4;
	}
	else if (gpio_pin == GPIO_Pin_5)
	{
		ret = EXTI_PinSource5;
	}
	else if (gpio_pin == GPIO_Pin_6)
	{
		ret = EXTI_PinSource6;
	}
	else if (gpio_pin == GPIO_Pin_7)
	{
		ret = EXTI_PinSource7;
	}
	else if (gpio_pin == GPIO_Pin_8)
	{
		ret = EXTI_PinSource8;
	}
	else if (gpio_pin == GPIO_Pin_9)
	{
		ret = EXTI_PinSource9;
	}
	else if (gpio_pin == GPIO_Pin_10)
	{
		ret = EXTI_PinSource10;
	}
	else if (gpio_pin == GPIO_Pin_11)
	{
		ret = EXTI_PinSource11;
	}
	else if (gpio_pin == GPIO_Pin_12)
	{
		ret = EXTI_PinSource12;
	}
	else if (gpio_pin == GPIO_Pin_13)
	{
		ret = EXTI_PinSource13;
	}
	else if (gpio_pin == GPIO_Pin_14)
	{
		ret = EXTI_PinSource14;
	}
	else if (gpio_pin == GPIO_Pin_15)
	{
		ret = EXTI_PinSource15;
	}

	return ret;
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_GetExitLine
*	����˵��: ���� GPIO_Pin_X �õ� EXTI_Linexx
*	��    �Σ�gpio_pin
*	�� �� ֵ: EXTI_Linexx
*********************************************************************************************************
*/
uint32_t bsp_GetExitLine(uint16_t gpio_pin)
{
	uint32_t ret;

	if (gpio_pin == GPIO_Pin_0)
	{
		ret = EXTI_Line0;
	}
	else if (gpio_pin == GPIO_Pin_1)
	{
		ret = EXTI_Line1;
	}
	else if (gpio_pin == GPIO_Pin_2)
	{
		ret = EXTI_Line2;
	}
	else if (gpio_pin == GPIO_Pin_3)
	{
		ret = EXTI_Line3;
	}
	else if (gpio_pin == GPIO_Pin_4)
	{
		ret = EXTI_Line4;
	}
	else if (gpio_pin == GPIO_Pin_5)
	{
		ret = EXTI_Line5;
	}
	else if (gpio_pin == GPIO_Pin_6)
	{
		ret = EXTI_Line6;
	}
	else if (gpio_pin == GPIO_Pin_7)
	{
		ret = EXTI_Line7;
	}
	else if (gpio_pin == GPIO_Pin_8)
	{
		ret = EXTI_Line8;
	}
	else if (gpio_pin == GPIO_Pin_9)
	{
		ret = EXTI_Line9;
	}
	else if (gpio_pin == GPIO_Pin_10)
	{
		ret = EXTI_Line10;
	}
	else if (gpio_pin == GPIO_Pin_11)
	{
		ret = EXTI_Line11;
	}
	else if (gpio_pin == GPIO_Pin_12)
	{
		ret = EXTI_Line12;
	}
	else if (gpio_pin == GPIO_Pin_13)
	{
		ret = EXTI_Line13;
	}
	else if (gpio_pin == GPIO_Pin_14)
	{
		ret = EXTI_Line14;
	}
	else if (gpio_pin == GPIO_Pin_15)
	{
		ret = EXTI_Line15;
	}

	return ret;
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_SetGpioExit
*	����˵��: ���������ⲿ�ж�
*	��    ��: 
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_SetGpioExit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, EXTITrigger_TypeDef EXIT_Trigger, IRQn_Type irq)
{
	EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_InitTypeDef   GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

	/*1. �������ŵ��ⲿ�жϲ�����*/
	/* Enable GPIOx clock */
	RCC_AHBPeriphClockCmd(bsp_GetRCCofGPIO(GPIOx), ENABLE);

	/* Configure pins as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOx, &GPIO_InitStructure);

	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Connect EXTI15 Line to PC15 pin */
	SYSCFG_EXTILineConfig(bsp_GetExitPortOfGPIO(GPIOx), bsp_GetExitPinSource(GPIO_Pin));

	/* Configure EXTIxx line */
	EXTI_InitStructure.EXTI_Line = bsp_GetExitLine(GPIO_Pin);  
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXIT_Trigger;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/*2. �����жϴ������Ѿ����ȼ�*/	
	/* Enable and set EXTI4_15 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = irq;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
}

/*
*********************************************************************************************************
*	�� �� ��: Stm32_Input_GPIO_Init()
*	����˵��: ���жϷ�ʽ��Ϊgpio��������ų�ʼ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/

void Stm32_Input_GPIO_Init(void)
{
	bsp_SetGpioExit(ROUTER_WAKE_648_PORT, ROUTER_WAKE_648_PIN, ROUTER_WAKE_648_EXIT_TRIGGER, ROUTER_WAKE_648_IRQ);
	bsp_SetGpioExit(WIFI_WAKE_648_PORT, WIFI_WAKE_648_PIN, WIFI_WAKE_648_EXIT_TRIGGER, WIFI_WAKE_648_IRQ);
	bsp_SetGpioExit(XFM10213_WAKE_648_PORT, XFM10213_WAKE_648_PIN, XFM10213_WAKE_648_EXIT_TRIGGER, XFM10213_WAKE_648_IRQ);
	bsp_SetGpioExit(MSTAR648_TO_STM32_GPIO2_PORT, MSTAR648_TO_STM32_GPIO2_PIN, MSTAR648_TO_STM32_GPIO2_EXIT_TRIGGER, MSTAR648_TO_STM32_GPIO2_IRQ);
	bsp_SetGpioExit(MSTAR648_TO_STM32_GPIO3_PORT, MSTAR648_TO_STM32_GPIO3_PIN, MSTAR648_TO_STM32_GPIO3_EXIT_TRIGGER, MSTAR648_TO_STM32_GPIO3_IRQ);	
}

void Stm32_Wake_Mstar648_Gpio_Init(void)
{
	GPIO_InitTypeDef        GPIO_InitStructure;
	/* GPIO Periph clock enable */
	RCC_AHBPeriphClockCmd(STM32_WAKE_648_CLK, ENABLE);

	/* Configure PCxx in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = STM32_WAKE_648_PIN;
	GPIO_InitStructure.GPIO_Mode = STM32_WAKE_648_MODE;
	GPIO_InitStructure.GPIO_OType = STM32_WAKE_648_OTYPE;
	GPIO_InitStructure.GPIO_Speed = STM32_WAKE_648_SPEED;
	GPIO_InitStructure.GPIO_PuPd = STM32_WAKE_648_PUPD;
	GPIO_Init(STM32_WAKE_648_PORT, &GPIO_InitStructure);	

	/*init gpio pin high level*/
	GPIO_SetBits(STM32_WAKE_648_PORT, STM32_WAKE_648_PIN);		
}

void XFM10213_Reset_Gpio_Init(void)
{
	GPIO_InitTypeDef        GPIO_InitStructure;
	/* GPIO Periph clock enable */
	RCC_AHBPeriphClockCmd(STM32_XFM10213_RESET_CLK, ENABLE);

	/* Configure PCxx in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = STM32_XFM10213_RESET_PIN;
	GPIO_InitStructure.GPIO_Mode = STM32_XFM10213_RESET_MODE;
	GPIO_InitStructure.GPIO_OType = STM32_XFM10213_RESET_OTYPE;
	GPIO_InitStructure.GPIO_Speed = STM32_XFM10213_RESET_SPEED;
	GPIO_InitStructure.GPIO_PuPd = STM32_XFM10213_RESET_PUPD;
	GPIO_Init(STM32_XFM10213_RESET_PORT, &GPIO_InitStructure);	

	/*init gpio pin low level*/
	GPIO_ResetBits(STM32_XFM10213_RESET_PORT, STM32_XFM10213_RESET_PIN);	
	
}

void I94113_Reset_Gpio_Init(void)
{
	GPIO_InitTypeDef        GPIO_InitStructure;
	/* GPIO Periph clock enable */
	RCC_AHBPeriphClockCmd(STM32_I94113_RESET_CLK, ENABLE);

	/* Configure PCxx in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = STM32_I94113_RESET_PIN;
	GPIO_InitStructure.GPIO_Mode = STM32_I94113_RESET_MODE;
	GPIO_InitStructure.GPIO_OType = STM32_I94113_RESET_OTYPE;
	GPIO_InitStructure.GPIO_Speed = STM32_I94113_RESET_SPEED;
	GPIO_InitStructure.GPIO_PuPd = STM32_I94113_RESET_PUPD;
	GPIO_Init(STM32_I94113_RESET_PORT, &GPIO_InitStructure);	

	/*init gpio pin high level*/
	GPIO_ResetBits(STM32_I94113_RESET_PORT, STM32_I94113_RESET_PIN);	
}

void Stm32_Output_To_648_Gpio_Init(void)
{
	struct s_gpioConfig gpio_init[] = {
		{STM32_TO_648_GPIO1_CLK, STM32_TO_648_GPIO1_PORT, {STM32_TO_648_GPIO1_PIN, STM32_TO_648_GPIO1_MODE, STM32_TO_648_GPIO1_SPEED, STM32_TO_648_GPIO1_OTYPE, STM32_TO_648_GPIO1_PUPD} },
	};
	int i = 0;
	GPIO_InitTypeDef        GPIO_InitStructure;
	
	for(i=0; i< ARRAY_SIZE(gpio_init); i++)
	{
		/* GPIO Periph clock enable */
		RCC_AHBPeriphClockCmd(gpio_init[i].Gpio_Clk, ENABLE);

		/* Configure PCxx in output pushpull mode */
		GPIO_InitStructure.GPIO_Pin = gpio_init[i].Gpio_Init_Config.GPIO_Pin;
		GPIO_InitStructure.GPIO_Mode = gpio_init[i].Gpio_Init_Config.GPIO_Mode;
		GPIO_InitStructure.GPIO_OType = gpio_init[i].Gpio_Init_Config.GPIO_OType;
		GPIO_InitStructure.GPIO_Speed = gpio_init[i].Gpio_Init_Config.GPIO_Speed;
		GPIO_InitStructure.GPIO_PuPd = gpio_init[i].Gpio_Init_Config.GPIO_PuPd;
		GPIO_Init(gpio_init[i].Gpio_Port, &GPIO_InitStructure);	

		/*init gpio pin low level*/
		GPIO_ResetBits(gpio_init[i].Gpio_Port, gpio_init[i].Gpio_Init_Config.GPIO_Pin);
	}
}

void GPIO_Pin_Init(void)
{
	uint32_t i =0;
	Stm32_Input_GPIO_Init();
	Stm32_Wake_Mstar648_Gpio_Init(); 
	XFM10213_Reset_Gpio_Init();
	I94113_Reset_Gpio_Init();
	Stm32_Output_To_648_Gpio_Init();
	
	XFM10213_RESET();
	I94113_RESET();
	for(i=0; i<100000; i++);//��ʱ100ms
	XFM10213_SET();
	I94113_SET();
}

void inline Stm32_Reset_XFM10213_Chip_Trigger(void)
{
	//__disable_irq();  /*�ر��ж�*/
	sg_reset_XFM10213_flag = 1;
	//__enable_irq();	  /*�����ж�*/
}

void Stm32_Reset_XFM10213_Chip_Loop(void)
{
	static __IO uint32_t loopCount = 0;
	if(sg_reset_XFM10213_flag)
	{
		if(loopCount == 0)
		{
			GPIO_ResetBits(STM32_XFM10213_RESET_PORT, STM32_XFM10213_RESET_PIN);
		}
		else if(loopCount > LOOP_COUNT) //ѭ��һ��ʱ���ִ�У����������ʱ��û��ȷ��Ҫ�󣬹���while��ѭ����������ʾһ��ʱ�䡣�л���ƽ
		{	
			loopCount = 0;
			
			//__disable_irq();  /*�ر��ж�*/
			sg_reset_XFM10213_flag = 0;
			//__enable_irq();	  /*�����ж�*/
			
			GPIO_SetBits(STM32_XFM10213_RESET_PORT, STM32_XFM10213_RESET_PIN);
			return;
		}	
		loopCount++;		
	}
}

void inline Stm32_Reset_I94113_Chip_Trigger(void)
{
	//__disable_irq();  /*�ر��ж�*/
	sg_reset_I94113_flag = 1;
	//__enable_irq();	  /*�����ж�*/
}

void Stm32_Reset_I94113_Chip_Loop(void)
{
	static __IO uint32_t loopCount = 0;
	if(sg_reset_I94113_flag)
	{
		if(loopCount == 0)
		{
			GPIO_ResetBits(STM32_I94113_RESET_PORT, STM32_I94113_RESET_PIN);
		}		  
		else if(loopCount > LOOP_COUNT) //ѭ��һ��ʱ���ִ�У����������ʱ��û��ȷ��Ҫ�󣬹���while��ѭ����������ʾһ��ʱ�䡣�л���ƽ
		{	
			loopCount = 0;
			
			//__disable_irq();    /*�ر��ж�*/
			sg_reset_I94113_flag = 0;
			//__enable_irq();	  /*�����ж�*/
			
			GPIO_SetBits(STM32_I94113_RESET_PORT, STM32_I94113_RESET_PIN);
			return;
		}	
		loopCount++;		
	}
}

void inline Stm32_Wake_Mstar648_Chip_Enable(void)
{
	//__disable_irq();    /*�ر��ж�*/
	sg_wake_648_flag = 1;
	//__enable_irq();	  /*�����ж�*/	
}

void inline Stm32_Wake_Mstar648_Chip_Disable(void)
{
	//__disable_irq();    /*�ر��ж�*/
	sg_wake_648_flag = 0;
	//__enable_irq();	  /*�����ж�*/
	
	GPIO_SetBits(STM32_WAKE_648_PORT, STM32_WAKE_648_PIN);
}

void Stm32_Wake_Mstar648_Chip_Loop(void)
{
	#define HIGH_LEVE_LOOP_COUNT   (-20)
	#define LOW_LEVELOOP_COUNT 	   (1000*1)
	static __IO int32_t loopCount = HIGH_LEVE_LOOP_COUNT;
	
	/*GPIO����Ĭ��Ϊ�ߵ�ƽ*/
	/*��Ҫ���ϵĲ���һ���½��ػ���648оƬ��ֱ��648оƬ����֮��֪ͨstm32ʱ�Źر�*/	
	if(sg_wake_648_flag)
	{
		if(loopCount == 0)
		{
			GPIO_ResetBits(STM32_WAKE_648_PORT, STM32_WAKE_648_PIN);
		}			
		else if(loopCount > LOW_LEVELOOP_COUNT) /* ѭ��һ��ʱ���ִ�У����������ʱ��û��ȷ��Ҫ�󣬹���while��ѭ����������ʾһ��ʱ�䡣�л���ƽ */
		{	
			loopCount = HIGH_LEVE_LOOP_COUNT;
			if(g_wifi_wake_flag == 1)  /* ����wifi���ѳ������� */
			{				
				GPIO_SetBits(STM32_WAKE_648_PORT, STM32_WAKE_648_PIN); 
			}
			else  /* ������wifi������ֻ����һ�� */
			{
				Stm32_Wake_Mstar648_Chip_Disable();				
			}			
			return;
		}				
		loopCount++;  
	}
	else
	{
		loopCount = HIGH_LEVE_LOOP_COUNT;
	}
}

void Stm32_Set_Gpio_Ouput(Gpio_Num_t number,Gpio_Level_t level)
{
	struct s_gpioConfig gpio_init[] = {
		{STM32_TO_648_GPIO1_CLK, STM32_TO_648_GPIO1_PORT, {STM32_TO_648_GPIO1_PIN, STM32_TO_648_GPIO1_MODE, STM32_TO_648_GPIO1_SPEED, STM32_TO_648_GPIO1_OTYPE, STM32_TO_648_GPIO1_PUPD} },
	};

	if(HIGH_LEVEL == level)
	{
		GPIO_SetBits(gpio_init[number].Gpio_Port, gpio_init[number].Gpio_Init_Config.GPIO_Pin);
	}
	else if(LOW_LEVEL == level)
	{
		GPIO_ResetBits(gpio_init[number].Gpio_Port, gpio_init[number].Gpio_Init_Config.GPIO_Pin);
	}
}

/**
  * @brief  This function handles External lines 4 to 15 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI4_15_IRQHandler(void)
{
	if(EXTI_GetITStatus(ROUTER_WAKE_648_LINE) != RESET)
	{
		/* Clear the EXTI linexx pending bit */
		EXTI_ClearITPendingBit(ROUTER_WAKE_648_LINE);	
		
		if(g_648_work_state == SLEEP_STATE)
		{
			LED_Open(LED1);		/* �����̵� */
			LED_Close(LED2);    /* �رպ�� */
			Stm32_Set_Gpio_Ouput(GPIO_NUM_1,LOW_LEVEL);   /* ����������־���Żָ�Ĭ��״̬ */
			Stm32_Wake_Mstar648_Chip_Enable();
		}
	}
	
	if(EXTI_GetITStatus(WIFI_WAKE_648_EXIT_LINE) != RESET)
	{
		/* Clear the EXTI linexx pending bit */
		EXTI_ClearITPendingBit(WIFI_WAKE_648_EXIT_LINE);	
		
		g_wifi_wake_flag = 1;
		Stm32_Set_Gpio_Ouput(GPIO_NUM_1,LOW_LEVEL);   /* ����������־���Żָ�Ĭ��״̬ */
		Stm32_Wake_Mstar648_Chip_Enable();	
		
		if(g_648_work_state == SLEEP_STATE)
		{
			LED_Open(LED1);		/* �����̵� */
			LED_Close(LED2);    /* �رպ�� */			
		}
	}

	if(EXTI_GetITStatus(XFM10213_WAKE_648_EXIT_LINE) != RESET)
	{
		/* Clear the EXTI linexx pending bit */
		EXTI_ClearITPendingBit(XFM10213_WAKE_648_EXIT_LINE);	

		xfm_irq_process();
	}

	if(EXTI_GetITStatus(MSTAR648_TO_STM32_GPIO2_EXIT_LINE) != RESET)
	{
		/* Clear the EXTI line 15 pending bit */
		EXTI_ClearITPendingBit(MSTAR648_TO_STM32_GPIO2_EXIT_LINE);
		
		/*�����ؽ����ж�,�ض���GPIO2�������ߣ���2��gpio״̬���ж�����Ҫ��ʱ�˲�����Ҫ������밴���߼���*/
		/*if ((GPIO_ReadInputDataBit(MSTAR648_TO_STM32_GPIO2_PORT, MSTAR648_TO_STM32_GPIO2_PIN) == 1) 
			&& (GPIO_ReadInputDataBit(MSTAR648_TO_STM32_GPIO3_PORT, MSTAR648_TO_STM32_GPIO3_PIN) == 0))*/
		//if(GPIO_ReadInputDataBit(MSTAR648_TO_STM32_GPIO2_PORT, MSTAR648_TO_STM32_GPIO2_PIN) == 1)
		{
			LED_Open(LED1);		/* �����̵� */
			LED_Close(LED2);    /* �رպ�� */
			bsp_ClearKey();		/* ��ջ���İ����¼� */
			if(1 == g_wifi_wake_flag)  /* ���wifi���ѱ�־ */
			{
				g_wifi_wake_flag = 0;
			}
			
			Stm32_Wake_Mstar648_Chip_Disable();			  /* �رջ�����������ķ��� */				
			g_648_work_state = RUN_STATE;						
		}
	}		
	
	if(EXTI_GetITStatus(MSTAR648_TO_STM32_GPIO3_EXIT_LINE) != RESET)
	{
		/* Clear the EXTI line 15 pending bit */
		EXTI_ClearITPendingBit(MSTAR648_TO_STM32_GPIO3_EXIT_LINE);
		
		/*�����ؽ����ж�,�ض���GPIO3�������ߣ���2��gpio״̬���ж�����Ҫ��ʱ�˲�����Ҫ������밴���߼���*/
		/*if ((GPIO_ReadInputDataBit(MSTAR648_TO_STM32_GPIO2_PORT, MSTAR648_TO_STM32_GPIO2_PIN) == 0) 
			&& (GPIO_ReadInputDataBit(MSTAR648_TO_STM32_GPIO3_PORT, MSTAR648_TO_STM32_GPIO3_PIN) == 1))*/
		//if(GPIO_ReadInputDataBit(MSTAR648_TO_STM32_GPIO3_PORT, MSTAR648_TO_STM32_GPIO3_PIN) == 1)
		{
			LED_Open(LED2);		/* ������� */
			LED_Close(LED1);    /* �ر��̵� */				
			bsp_ClearKey();		/* ��ջ���İ����¼� */	
			if((sg_wake_648_flag == 1)  && (g_wifi_wake_flag != 1)) /* ����wifi����gpio����رջ�����������ķ��� */
			{
				Stm32_Wake_Mstar648_Chip_Disable();  
			}					
			g_648_work_state = SLEEP_STATE;											
		}
	}
}

