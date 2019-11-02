#ifndef __GPIO_H
#define	__GPIO_H

#include "stm32f0xx.h"

//stm32 gpio output pin define
#define STM32_TO_648_GPIO1_PORT			GPIOC
#define STM32_TO_648_GPIO1_PIN			GPIO_Pin_13
#define STM32_TO_648_GPIO1_MODE			GPIO_Mode_OUT
#define STM32_TO_648_GPIO1_SPEED		GPIO_Speed_Level_3
#define STM32_TO_648_GPIO1_OTYPE		GPIO_OType_PP
#define STM32_TO_648_GPIO1_PUPD			GPIO_PuPd_DOWN
#define STM32_TO_648_GPIO1_CLK			RCC_AHBPeriph_GPIOC
 
#define STM32_I94113_RESET_PORT			GPIOA
#define STM32_I94113_RESET_PIN			GPIO_Pin_7
#define STM32_I94113_RESET_MODE			GPIO_Mode_OUT
#define STM32_I94113_RESET_SPEED		GPIO_Speed_Level_3
#define STM32_I94113_RESET_OTYPE		GPIO_OType_PP
#define STM32_I94113_RESET_PUPD			GPIO_PuPd_DOWN
#define STM32_I94113_RESET_CLK			RCC_AHBPeriph_GPIOA

#define STM32_WAKE_648_PORT				GPIOB
#define STM32_WAKE_648_PIN				GPIO_Pin_2
#define STM32_WAKE_648_MODE				GPIO_Mode_OUT
#define STM32_WAKE_648_SPEED			GPIO_Speed_Level_3
#define STM32_WAKE_648_OTYPE			GPIO_OType_PP
#define STM32_WAKE_648_PUPD				GPIO_PuPd_UP
#define STM32_WAKE_648_CLK				RCC_AHBPeriph_GPIOB

#define STM32_XFM10213_RESET_PORT		GPIOB
#define STM32_XFM10213_RESET_PIN		GPIO_Pin_9
#define STM32_XFM10213_RESET_MODE		GPIO_Mode_OUT
#define STM32_XFM10213_RESET_SPEED		GPIO_Speed_Level_3
#define STM32_XFM10213_RESET_OTYPE		GPIO_OType_PP
#define STM32_XFM10213_RESET_PUPD		GPIO_PuPd_DOWN
#define STM32_XFM10213_RESET_CLK		RCC_AHBPeriph_GPIOB

//stm32 gpio input pin define
#define KEY_WAKE_648_PORT				GPIOB
#define KEY_WAKE_648_PIN				GPIO_Pin_13
#define KEY_WAKE_648_CLK				RCC_AHBPeriph_GPIOB

#define ROUTER_WAKE_648_PORT			GPIOF
#define ROUTER_WAKE_648_PIN				GPIO_Pin_6
#define ROUTER_WAKE_648_CLK				RCC_AHBPeriph_GPIOF
#define ROUTER_WAKE_648_LINE			EXTI_Line6
#define ROUTER_WAKE_648_EXIT_TRIGGER	EXTI_Trigger_Falling
#define ROUTER_WAKE_648_IRQ				EXTI4_15_IRQn

#define WIFI_WAKE_648_PORT				GPIOF
#define WIFI_WAKE_648_PIN				GPIO_Pin_7
#define WIFI_WAKE_648_CLK				RCC_AHBPeriph_GPIOF
#define WIFI_WAKE_648_EXIT_LINE			EXTI_Line7
#define WIFI_WAKE_648_EXIT_TRIGGER		EXTI_Trigger_Falling
#define WIFI_WAKE_648_IRQ				EXTI4_15_IRQn

#define XFM10213_WAKE_648_PORT			GPIOB
#define XFM10213_WAKE_648_PIN			GPIO_Pin_8
#define XFM10213_WAKE_648_CLK			RCC_AHBPeriph_GPIOB
#define XFM10213_WAKE_648_EXIT_LINE		EXTI_Line8
#define XFM10213_WAKE_648_EXIT_TRIGGER	EXTI_Trigger_Rising
#define XFM10213_WAKE_648_IRQ			EXTI4_15_IRQn

#define MSTAR648_TO_STM32_GPIO2_PORT	GPIOC
#define MSTAR648_TO_STM32_GPIO2_PIN		GPIO_Pin_14
#define MSTAR648_TO_STM32_GPIO2_MODE	GPIO_Mode_IN
#define MSTAR648_TO_STM32_GPIO2_SPEED	GPIO_Speed_Level_3
#define MSTAR648_TO_STM32_GPIO2_OTYPE	GPIO_OType_PP
#define MSTAR648_TO_STM32_GPIO2_PUPD	GPIO_PuPd_NOPULL
#define MSTAR648_TO_STM32_GPIO2_CLK		RCC_AHBPeriph_GPIOC
#define MSTAR648_TO_STM32_GPIO2_EXIT_PROT		EXTI_PortSourceGPIOC
#define MSTAR648_TO_STM32_GPIO2_EXIT_PIN		EXTI_PinSource14
#define MSTAR648_TO_STM32_GPIO2_EXIT_LINE		EXTI_Line14
#define MSTAR648_TO_STM32_GPIO2_EXIT_TRIGGER	EXTI_Trigger_Rising
#define MSTAR648_TO_STM32_GPIO2_IRQ				EXTI4_15_IRQn

#define MSTAR648_TO_STM32_GPIO3_PORT			GPIOC
#define MSTAR648_TO_STM32_GPIO3_PIN				GPIO_Pin_15
#define MSTAR648_TO_STM32_GPIO3_MODE			GPIO_Mode_IN
#define MSTAR648_TO_STM32_GPIO3_PUPD			GPIO_PuPd_NOPULL
#define MSTAR648_TO_STM32_GPIO3_CLK				RCC_AHBPeriph_GPIOC
#define MSTAR648_TO_STM32_GPIO3_EXIT_PROT		EXTI_PortSourceGPIOC
#define MSTAR648_TO_STM32_GPIO3_EXIT_PIN		EXTI_PinSource15
#define MSTAR648_TO_STM32_GPIO3_EXIT_LINE		EXTI_Line15
#define MSTAR648_TO_STM32_GPIO3_EXIT_TRIGGER	EXTI_Trigger_Rising
#define MSTAR648_TO_STM32_GPIO3_IRQ				EXTI4_15_IRQn


#define ARRAY_SIZE(x) (sizeof(x)/sizeof((x)[0]))
	
#define XFM10213_RESET() 	GPIO_ResetBits(STM32_XFM10213_RESET_PORT, STM32_XFM10213_RESET_PIN)
#define XFM10213_SET() 		GPIO_SetBits(STM32_XFM10213_RESET_PORT, STM32_XFM10213_RESET_PIN)

#define I94113_RESET()    	GPIO_ResetBits(STM32_I94113_RESET_PORT, STM32_I94113_RESET_PIN)
#define I94113_SET() 	    GPIO_SetBits(STM32_I94113_RESET_PORT, STM32_I94113_RESET_PIN)

//2000�λ���1ms
#define LOOP_COUNT 	   (2000*1)

/* struct define*/
struct s_gpioConfig {
	uint32_t			Gpio_Clk;
	GPIO_TypeDef *		Gpio_Port;
	GPIO_InitTypeDef 	Gpio_Init_Config;	
};

typedef enum {
	GPIO_NUM_1=0,
	//GPIO_NUM_2,
	//GPIO_NUM_3,	
} Gpio_Num_t;

typedef enum {
	HIGH_LEVEL=0,
	LOW_LEVEL,
} Gpio_Level_t;

typedef enum {
	SLEEP_STATE=0,
	RUN_STATE,
} E_Mstar648WorkState;

extern __IO uint8_t g_648_work_state;
extern __IO uint8_t g_wifi_wake_flag;

//function define
//gpio init
void GPIO_Pin_Init(void); 					/* ֱ�ӵ���gpioģ������г�ʼ��������*/
//void Mstar648_Wake_Source_Gpio_Init(void);  /* gpio 4·���룬��ģ��Ϊkey��ʹ�ð���ɨ�跽ʽ����*/
//void Mstar648_Work_State_Gpio_Init(void);	/*Mstar����״ָ̬ʾ����GPIOI��ʼ����GPIO2��λ��GPIO3��λ��00Ĭ�ϣ�01������10˯�ߣ�11����*/
void Stm32_Input_GPIO_Init(void);			/*����gpio��ʼ��������key*/
void Stm32_Wake_Mstar648_Gpio_Init(void);   /* �����648 ��SAR0����Ҫ���һ�������½��ص����塣�ȴ�Mstar����֮��֪ͨstm32��ֹͣ���*/
void XFM10213_Reset_Gpio_Init(void);		/* stm32��λXFM10213оƬ���ų�ʼ�� */	
void I94113_Reset_Gpio_Init(void);			/* stm32��λI94113оƬ���ų�ʼ�� */	
void Stm32_Output_To_648_Gpio_Init(void);	/* stm32����ź�֪ͨmstar648���ų�ʼ�� */

//gpin function
/* stm32������λXFM10213оƬʹ�� */	
void Stm32_Reset_XFM10213_Chip_Trigger(void); 
/*  ѭ��ִ�к�������Ҫ��main��while�е��ã������ô���ʹ��ʱ��stm32���һ���½�������λXFM10213оƬ */	
void Stm32_Reset_XFM10213_Chip_Loop(void);    

/* stm32������λI94113оƬʹ�� */	
void Stm32_Reset_I94113_Chip_Trigger(void);	  
/* ѭ��ִ�к�������Ҫ��main��while�е��ã� �����ô���ʹ��ʱ��stm32���һ���½�������λI94113оƬ */	
void Stm32_Reset_I94113_Chip_Loop(void);	 

/* stm32����648ʹ�� */
void Stm32_Wake_Mstar648_Chip_Enable(void);
/* stm32����648�ر� */
void Stm32_Wake_Mstar648_Chip_Disable(void);
/* ������ʹ�ܺ�ѭ�����������½��أ�ֱ�����ùرպ��� */
void Stm32_Wake_Mstar648_Chip_Loop(void);

/*����GPIO���*/
void Stm32_Set_Gpio_Ouput(Gpio_Num_t number,Gpio_Level_t level);

//intrerupt function
void EXTI4_15_IRQHandler(void);

#endif /*__GPIO_H*/
