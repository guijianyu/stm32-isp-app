#include "led.h"

/**********************************************/
/* �������ܣ� led�� ��ʼ��                     */
/* ��ڲ�������                               */
/**********************************************/
void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_AHBPeriphClockCmd(LED1_CLK | LED2_CLK, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin = LED1_PIN ;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed =GPIO_Speed_Level_3;
	GPIO_Init(LED1_PORT, &GPIO_InitStruct);
	GPIO_ResetBits(LED1_PORT, LED1_PIN );
	
	GPIO_InitStruct.GPIO_Pin = LED2_PIN ;
	GPIO_Init(LED2_PORT, &GPIO_InitStruct);
	GPIO_ResetBits(LED2_PORT, LED2_PIN );
}

/**********************************************/
/* �������ܣ���led��                        */
/* ��ڲ�������                               */
/**********************************************/
void LED_Open(LedNum_t num)
{
	if(LED1 == num)
	{
		GPIO_SetBits(LED1_PORT, LED1_PIN );
	}
	else if (LED2 == num)
	{
		GPIO_SetBits(LED2_PORT, LED2_PIN );
	}
}
/**********************************************/
/* �������ܣ� �ص�led��                        */
/* ��ڲ�������                               */
/**********************************************/
void LED_Close(LedNum_t num)
{
	if(LED1 == num)
	{
		GPIO_ResetBits(LED1_PORT, LED1_PIN );	
	}
	else if (LED2 == num)
	{
		GPIO_ResetBits(LED2_PORT, LED2_PIN );
	}	
}

/**********************************************/
/* �������ܣ�led��ת                          */
/* ��ڲ�������                               */
/**********************************************/
void LED_Toggle(LedNum_t num)
{	
	if(LED1 == num)
	{
		GPIO_WriteBit(LED1_PORT, LED1_PIN, 
				   (BitAction)((1-GPIO_ReadOutputDataBit(LED1_PORT, LED1_PIN))));	
	}
	else if (LED2 == num)
	{
		GPIO_WriteBit(LED2_PORT, LED2_PIN, 
				   (BitAction)((1-GPIO_ReadOutputDataBit(LED2_PORT, LED2_PIN))));	;
	}		
}
