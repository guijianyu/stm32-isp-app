#include "bsp_delay.h"

/* 这个全局变量转用于 bsp_DelayMS() 函数 */
volatile uint32_t g_uiDelayCount = 0;

/*
*********************************************************************************************************
*	函 数 名: bsp_DelayMS
*	功能说明: ms级延迟，延迟精度为正负1ms
*	形    参:  n : 延迟长度，单位1 ms。 n 应大于2
*	返 回 值: 无
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

	//__disable_irq();  					/* 关中断 */
	g_uiDelayCount = n;						/* systick中断每1ms进入一次 */
	//__enable_irq();   					/* 开中断 */

	/*
		等待延迟时间到
		注意：编译器认为 g_uiDelayCount = 0，所以可能优化错误，因此 g_uiDelayCount 变量必须申明为 volatile
	*/
	while (g_uiDelayCount != 0);
}

/*
*********************************************************************************************************
*    函 数 名: bsp_DelayUS
*    功能说明: us级延迟。 必须在systick定时器启动后才能调用此函数。
*    形    参:  n : 延迟长度，单位1 us
*    返 回 值: 无
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
    ticks = n * (SystemCoreClock / 1000000);	 /* 需要的节拍数 */  
    
    tcnt = 0;
    told = SysTick->VAL;             /* 刚进入时的计数器值 */

    while (1)
    {
        tnow = SysTick->VAL;    
        if (tnow != told)
        {    
            /* SYSTICK是一个递减的计数器 */    
            if (tnow < told)
            {
                tcnt += told - tnow;    
            }
            /* 重新装载递减 */
            else
            {
                tcnt += reload - tnow + told;    
            }        
            told = tnow;

            /* 时间超过/等于要延迟的时间,则退出 */
            if (tcnt >= ticks)
            {
            	break;
            }
        }  
    }
} 

