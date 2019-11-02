#include "bsp_delay.h"

/* ���ȫ�ֱ���ת���� bsp_DelayMS() ���� */
volatile uint32_t g_uiDelayCount = 0;

/*
*********************************************************************************************************
*	�� �� ��: bsp_DelayMS
*	����˵��: ms���ӳ٣��ӳپ���Ϊ����1ms
*	��    ��:  n : �ӳٳ��ȣ���λ1 ms�� n Ӧ����2
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_DelayMS(__IO uint32_t n)
{	
	if (n == 0)
	{
		return;
	}
	else if (n == 1)
	{
		n = 2; 
	}

	//__disable_irq();  					/* ���ж� */
	g_uiDelayCount = n;						/* systick�ж�ÿ1ms����һ�� */
	//__enable_irq();   					/* ���ж� */

	/*
		�ȴ��ӳ�ʱ�䵽
		ע�⣺��������Ϊ g_uiDelayCount = 0�����Կ����Ż�������� g_uiDelayCount ������������Ϊ volatile
	*/
	while (g_uiDelayCount != 0);
}

/*
*********************************************************************************************************
*    �� �� ��: bsp_DelayUS
*    ����˵��: us���ӳ١� ������systick��ʱ����������ܵ��ô˺�����
*    ��    ��:  n : �ӳٳ��ȣ���λ1 us
*    �� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_DelayUS(uint32_t n)
{
    uint32_t ticks;
    uint32_t told;
    uint32_t tnow;
    uint32_t tcnt = 0;
    uint32_t reload;
       
	reload = SysTick->LOAD;                
    ticks = n * (SystemCoreClock / 1000000);	 /* ��Ҫ�Ľ����� */  
    
    tcnt = 0;
    told = SysTick->VAL;             /* �ս���ʱ�ļ�����ֵ */

    while (1)
    {
        tnow = SysTick->VAL;    
        if (tnow != told)
        {    
            /* SYSTICK��һ���ݼ��ļ����� */    
            if (tnow < told)
            {
                tcnt += told - tnow;    
            }
            /* ����װ�صݼ� */
            else
            {
                tcnt += reload - tnow + told;    
            }        
            told = tnow;

            /* ʱ�䳬��/����Ҫ�ӳٵ�ʱ��,���˳� */
            if (tcnt >= ticks)
            {
            	break;
            }
        }  
    }
} 

