#include "led.h"

/**********************************************/
/* �������ܣ� led�� ��ʼ��                     */
/* ��ڲ�������                               */
/**********************************************/
void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	GPIO_InitStruct.GPIO_Pin = LED1_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed =GPIO_Speed_Level_3;
	GPIO_Init(LED1_PORT, &GPIO_InitStruct);
	GPIO_ResetBits(LED1_PORT, LED1_PIN);
}

/**********************************************/
/* �������ܣ���led��                        */
/* ��ڲ�������                               */
/**********************************************/
void LED_Open(void)
{
	 GPIO_ResetBits(LED1_PORT, LED1_PIN );
}
/**********************************************/
/* �������ܣ� �ص�led��                        */
/* ��ڲ�������                               */
/**********************************************/
void LED_Close(void)
{
	GPIO_SetBits(LED1_PORT, LED1_PIN );
}

/**********************************************/
/* �������ܣ�led��ת                          */
/* ��ڲ�������                               */
/**********************************************/
void LED_Toggle(void)
{
		GPIO_WriteBit(LED1_PORT, LED1_PIN, 
					   (BitAction)((1-GPIO_ReadOutputDataBit(LED1_PORT, LED1_PIN))));
}
