#ifndef __BSP_KEY_H
#define	__BSP_KEY_H

#include "stm32f0xx.h"

/*
	把所有的唤醒源当作按键事件处理
*/
#define WAKE_SOURCE_COUNT    1	    /*唤醒源的个数*/ 	

/*
	按键滤波时间10ms, 单位10ms。
	只有连续检测到50ms状态不变才认为有效，包括弹起和按下两种事件
	即使按键电路不做硬件滤波，该滤波机制也可以保证可靠地检测到按键事件
*/
#define KEY_FILTER_TIME   1

/* 按键ID, 主要用于bsp_KeyState()函数的入口参数 */
typedef enum
{
	KID_K1 = 0,
}KEY_ID_E;

/*
	定义键值代码, 必须按如下次序定时每个键的按下、弹起和长按事件

	推荐使用enum, 不用#define，原因：
	(1) 便于新增键值,方便调整顺序，使代码看起来舒服点
	(2) 编译器可帮我们避免键值重复。
*/
typedef enum
{
	KEY_NONE = 0,					/* 0 表示按键事件 */

	KEY_1_DOWN,						/* 1键按下 */
	KEY_1_UP,						/* 1键弹起 */
}KEY_ENUM;


/* 按键FIFO用到变量 */
#define KEY_FIFO_SIZE	10
typedef struct
{
	uint8_t Buf[KEY_FIFO_SIZE];		/* 键值缓冲区 */
	uint8_t Read;					/* 缓冲区读指针1 */
	uint8_t Write;					/* 缓冲区写指针 */
}KEY_FIFO_T;

/*
	每个按键对应1个全局的结构体变量。
*/
typedef struct
{
	/* 下面是一个函数指针，指向判断按键手否按下的函数 */
	uint8_t (*IsKeyDownFunc)(void); /* 按键按下的判断函数,1表示按下 */

	uint8_t  Count;			/* 滤波器计数器 */
	uint8_t  State;			/* 按键当前状态（按下还是弹起） */
}KEY_T;

/*所有的唤醒事件认为是按键时间，唤醒按键扫描*/
void bsp_key_scan(void);
void bsp_key_event_process(void);

/* 供外部调用的函数声明 */
void bsp_InitKey(void);
void bsp_PutKey(uint8_t _KeyCode);
uint8_t bsp_GetKey(void);
uint8_t bsp_GetKeyState(KEY_ID_E _ucKeyID);
void bsp_ClearKey(void);

#endif  //__BSP_KEY_H
