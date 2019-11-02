#include <stdio.h>
#include "bsp_key.h"
#include "gpio.h"
#include "led.h"

static KEY_T s_tBtn[WAKE_SOURCE_COUNT];	/*每个按键对应1个全局的结构体变量*/
static KEY_FIFO_T s_tKey; /* 按键FIFO 的结构体变量 */

__attribute__((unused)) static uint8_t IsKeyDown(void) 				{if (GPIO_ReadInputDataBit(KEY_WAKE_648_PORT, KEY_WAKE_648_PIN) == 0) return 1;else return 0;}
__attribute__((unused)) static uint8_t IsRouterDown(void) 			{if (GPIO_ReadInputDataBit(ROUTER_WAKE_648_PORT, ROUTER_WAKE_648_PIN) == 0) return 1;else return 0;}
__attribute__((unused)) static uint8_t IsWifiDown(void) 			{if (GPIO_ReadInputDataBit(WIFI_WAKE_648_PORT, WIFI_WAKE_648_PIN) == 0) return 1;else return 0;}
__attribute__((unused)) static uint8_t IsXFM10213Down(void) 		{if (GPIO_ReadInputDataBit(XFM10213_WAKE_648_PORT, XFM10213_WAKE_648_PIN) == 1) return 1;else return 0;}

/*组合按键*/
__attribute__((unused)) static uint8_t IsMstarRunDown(void)	
{
	if ((GPIO_ReadInputDataBit(MSTAR648_TO_STM32_GPIO2_PORT, MSTAR648_TO_STM32_GPIO2_PIN) == 1) 
		&& (GPIO_ReadInputDataBit(MSTAR648_TO_STM32_GPIO3_PORT, MSTAR648_TO_STM32_GPIO3_PIN) == 0))
		return 1;
	else
		return 0;
}
__attribute__((unused)) static uint8_t IsMstarSleepDown(void) 
{
	if ((GPIO_ReadInputDataBit(MSTAR648_TO_STM32_GPIO2_PORT, MSTAR648_TO_STM32_GPIO2_PIN) == 0) 
		&& (GPIO_ReadInputDataBit(MSTAR648_TO_STM32_GPIO3_PORT, MSTAR648_TO_STM32_GPIO3_PIN) == 1))
		return 1;
	else
		return 0;
}

/*
*********************************************************************************************************
*	函 数 名: bsp_PutKey
*	功能说明: 将1个键值压入按键FIFO缓冲区。
*	形    参:  _KeyCode : 按键代码
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_PutKey(uint8_t _KeyCode)
{
	s_tKey.Buf[s_tKey.Write] = _KeyCode;

	if (++s_tKey.Write  >= KEY_FIFO_SIZE)
	{
		s_tKey.Write = 0;
	}
}

/*
*********************************************************************************************************
*	函 数 名: bsp_GetKey
*	功能说明: 从按键FIFO缓冲区读取一个键值。
*	形    参:  无
*	返 回 值: 按键代码
*********************************************************************************************************
*/
uint8_t bsp_GetKey(void)
{
	uint8_t ret;

	if (s_tKey.Read == s_tKey.Write)
	{
		return KEY_NONE;
	}
	else
	{
		ret = s_tKey.Buf[s_tKey.Read];

		if (++s_tKey.Read >= KEY_FIFO_SIZE)
		{
			s_tKey.Read = 0;
		}
		return ret;
	}
}

/*
*********************************************************************************************************
*	函 数 名: bsp_GetKeyState
*	功能说明: 读取按键的状态
*	形    参:  _ucKeyID : 按键ID，从0开始
*	返 回 值: 1 表示按下， 0 表示未按下
*********************************************************************************************************
*/
uint8_t bsp_GetKeyState(KEY_ID_E _ucKeyID)
{
	return s_tBtn[_ucKeyID].State;
}

/*
*********************************************************************************************************
*	函 数 名: bsp_ClearKey
*	功能说明: 清空按键FIFO缓冲区
*	形    参：无
*	返 回 值: 按键代码
*********************************************************************************************************
*/
void bsp_ClearKey(void)
{
	s_tKey.Read = s_tKey.Write;
}

/*
*********************************************************************************************************
*	函 数 名: bsp_InitKeyVar
*	功能说明: 初始化按键变量
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_InitKeyVar(void)
{
	uint8_t i;

	/* 对按键FIFO读写指针清零 */
	s_tKey.Read = 0;
	s_tKey.Write = 0;

	/* 给每个按键结构体成员变量赋一组缺省值 */
	for (i = 0; i < WAKE_SOURCE_COUNT; i++)
	{
		s_tBtn[i].Count = KEY_FILTER_TIME / 2;			/* 计数器设置为滤波时间的一半 */
		s_tBtn[i].State = 0;							/* 按键缺省状态，0为未按下 */
	}

	/* 判断按键按下的函数 */
	s_tBtn[KID_K1].IsKeyDownFunc = IsKeyDown;
	
	/*
	s_tBtn[KID_ROUTER].IsKeyDownFunc = IsRouterDown;
	s_tBtn[KID_WIFI].IsKeyDownFunc = IsWifiDown;
	s_tBtn[KID_XFM12013].IsKeyDownFunc = IsXFM10213Down;
	s_tBtn[KID_MSTAR_RUN].IsKeyDownFunc = IsMstarRunDown;
	s_tBtn[KID_MSTAR_SLEEP].IsKeyDownFunc = IsMstarSleepDown;
	*/
}

/*
*********************************************************************************************************
*	函 数 名: bsp_DetectKey
*	功能说明: 检测一个按键。非阻塞状态，必须被周期性的调用。
*	形    参:  按键结构变量指针
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_DetectKey(uint8_t i)
{
	KEY_T *pBtn;

	/*
		如果没有初始化按键函数，则报错
	*/
	if (s_tBtn[i].IsKeyDownFunc == 0)
	{
		printf("Fault : DetectButton(), s_tBtn[i].IsKeyDownFunc undefine");
		return ; 
	}
		
	pBtn = &s_tBtn[i];
	if (pBtn->IsKeyDownFunc())
	{
		if (pBtn->Count < KEY_FILTER_TIME)
		{
			pBtn->Count = KEY_FILTER_TIME;
		}
		else if(pBtn->Count < 2 * KEY_FILTER_TIME)
		{
			pBtn->Count++;
		}
		else
		{
			if (pBtn->State == 0)
			{
				pBtn->State = 1;

				/* 发送按钮按下的消息 */
				bsp_PutKey((uint8_t)(2 * i + 1));
			}
		}
	}
	else
	{
		if(pBtn->Count > KEY_FILTER_TIME)
		{
			pBtn->Count = KEY_FILTER_TIME;
		}
		else if(pBtn->Count != 0)
		{
			pBtn->Count--;
		}
		else
		{
			if (pBtn->State == 1)
			{
				pBtn->State = 0;

				/* 发送按钮弹起的消息 */
				bsp_PutKey((uint8_t)(2 * i + 2));
			}
		}
	}
}

/*
*********************************************************************************************************
*	函 数 名: bsp_KeyScan
*	功能说明: 扫描所有按键。非阻塞，被systick中断，10ms周期性的调用
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_key_scan(void)
{
	uint8_t i;

	for (i = 0; i < WAKE_SOURCE_COUNT; i++)
	{
		bsp_DetectKey(i);
	}
}

/*
*********************************************************************************************************
*	函 数 名: bsp_InitKeyHard
*	功能说明: 配置按键对应的GPIO
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_InitKeyHard(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 第1步：打开GPIO时钟 */
	RCC_AHBPeriphClockCmd(KEY_WAKE_648_CLK, ENABLE);

	/* 第2步：配置按键GPIO为浮动输入模式(实际上CPU复位后就是输入状态) */
	/* Configure pins as input floating */
	GPIO_InitStructure.GPIO_Pin = KEY_WAKE_648_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(KEY_WAKE_648_PORT, &GPIO_InitStructure);
}

/*
*********************************************************************************************************
*	函 数 名: bsp_InitWakeSource
*	功能说明: 按键初始化
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitKey(void)
{
	/* 初始化按键硬件 */
	bsp_InitKeyHard();
	
	/* 初始化按键变量 */
	bsp_InitKeyVar();		
}

void bsp_key_event_process(void)
{
	uint8_t ucKeyCode;		
	
	/* 按键滤波和检测由后台systick中断服务程序实现，我们只需要调用bsp_GetKey读取键值即可。 */
	ucKeyCode = bsp_GetKey();	/* 读取键值, 无键按下时返回 KEY_NONE = 0 */	
	if (ucKeyCode != KEY_NONE)
	{
		switch(ucKeyCode)
		{
			case KEY_1_DOWN:
				#ifdef DEBUG_TEST
				printf("whh test key\r\n");
				#endif
				if(g_648_work_state == SLEEP_STATE)
				{
					LED_Open(LED1);		/* 开启绿灯 */
					LED_Close(LED2);    /* 关闭红灯 */
					Stm32_Set_Gpio_Ouput(GPIO_NUM_1,LOW_LEVEL);   /* 音乐启动标志引脚恢复默认状态 */
				}				
				Stm32_Wake_Mstar648_Chip_Enable();
				break;
			default :
				break;
		}
	}
}
