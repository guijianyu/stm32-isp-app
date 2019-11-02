#include <stdio.h>
#include "bsp_key.h"
#include "gpio.h"
#include "led.h"

static KEY_T s_tBtn[WAKE_SOURCE_COUNT];	/*ÿ��������Ӧ1��ȫ�ֵĽṹ�����*/
static KEY_FIFO_T s_tKey; /* ����FIFO �Ľṹ����� */

__attribute__((unused)) static uint8_t IsKeyDown(void) 				{if (GPIO_ReadInputDataBit(KEY_WAKE_648_PORT, KEY_WAKE_648_PIN) == 0) return 1;else return 0;}
__attribute__((unused)) static uint8_t IsRouterDown(void) 			{if (GPIO_ReadInputDataBit(ROUTER_WAKE_648_PORT, ROUTER_WAKE_648_PIN) == 0) return 1;else return 0;}
__attribute__((unused)) static uint8_t IsWifiDown(void) 			{if (GPIO_ReadInputDataBit(WIFI_WAKE_648_PORT, WIFI_WAKE_648_PIN) == 0) return 1;else return 0;}
__attribute__((unused)) static uint8_t IsXFM10213Down(void) 		{if (GPIO_ReadInputDataBit(XFM10213_WAKE_648_PORT, XFM10213_WAKE_648_PIN) == 1) return 1;else return 0;}

/*��ϰ���*/
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
*	�� �� ��: bsp_PutKey
*	����˵��: ��1����ֵѹ�밴��FIFO��������
*	��    ��:  _KeyCode : ��������
*	�� �� ֵ: ��
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
*	�� �� ��: bsp_GetKey
*	����˵��: �Ӱ���FIFO��������ȡһ����ֵ��
*	��    ��:  ��
*	�� �� ֵ: ��������
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
*	�� �� ��: bsp_GetKeyState
*	����˵��: ��ȡ������״̬
*	��    ��:  _ucKeyID : ����ID����0��ʼ
*	�� �� ֵ: 1 ��ʾ���£� 0 ��ʾδ����
*********************************************************************************************************
*/
uint8_t bsp_GetKeyState(KEY_ID_E _ucKeyID)
{
	return s_tBtn[_ucKeyID].State;
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_ClearKey
*	����˵��: ��հ���FIFO������
*	��    �Σ���
*	�� �� ֵ: ��������
*********************************************************************************************************
*/
void bsp_ClearKey(void)
{
	s_tKey.Read = s_tKey.Write;
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_InitKeyVar
*	����˵��: ��ʼ����������
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_InitKeyVar(void)
{
	uint8_t i;

	/* �԰���FIFO��дָ������ */
	s_tKey.Read = 0;
	s_tKey.Write = 0;

	/* ��ÿ�������ṹ���Ա������һ��ȱʡֵ */
	for (i = 0; i < WAKE_SOURCE_COUNT; i++)
	{
		s_tBtn[i].Count = KEY_FILTER_TIME / 2;			/* ����������Ϊ�˲�ʱ���һ�� */
		s_tBtn[i].State = 0;							/* ����ȱʡ״̬��0Ϊδ���� */
	}

	/* �жϰ������µĺ��� */
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
*	�� �� ��: bsp_DetectKey
*	����˵��: ���һ��������������״̬�����뱻�����Եĵ��á�
*	��    ��:  �����ṹ����ָ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_DetectKey(uint8_t i)
{
	KEY_T *pBtn;

	/*
		���û�г�ʼ�������������򱨴�
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

				/* ���Ͱ�ť���µ���Ϣ */
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

				/* ���Ͱ�ť�������Ϣ */
				bsp_PutKey((uint8_t)(2 * i + 2));
			}
		}
	}
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_KeyScan
*	����˵��: ɨ�����а���������������systick�жϣ�10ms�����Եĵ���
*	��    ��:  ��
*	�� �� ֵ: ��
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
*	�� �� ��: bsp_InitKeyHard
*	����˵��: ���ð�����Ӧ��GPIO
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_InitKeyHard(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ��1������GPIOʱ�� */
	RCC_AHBPeriphClockCmd(KEY_WAKE_648_CLK, ENABLE);

	/* ��2�������ð���GPIOΪ��������ģʽ(ʵ����CPU��λ���������״̬) */
	/* Configure pins as input floating */
	GPIO_InitStructure.GPIO_Pin = KEY_WAKE_648_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(KEY_WAKE_648_PORT, &GPIO_InitStructure);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_InitWakeSource
*	����˵��: ������ʼ��
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitKey(void)
{
	/* ��ʼ������Ӳ�� */
	bsp_InitKeyHard();
	
	/* ��ʼ���������� */
	bsp_InitKeyVar();		
}

void bsp_key_event_process(void)
{
	uint8_t ucKeyCode;		
	
	/* �����˲��ͼ���ɺ�̨systick�жϷ������ʵ�֣�����ֻ��Ҫ����bsp_GetKey��ȡ��ֵ���ɡ� */
	ucKeyCode = bsp_GetKey();	/* ��ȡ��ֵ, �޼�����ʱ���� KEY_NONE = 0 */	
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
					LED_Open(LED1);		/* �����̵� */
					LED_Close(LED2);    /* �رպ�� */
					Stm32_Set_Gpio_Ouput(GPIO_NUM_1,LOW_LEVEL);   /* ����������־���Żָ�Ĭ��״̬ */
				}				
				Stm32_Wake_Mstar648_Chip_Enable();
				break;
			default :
				break;
		}
	}
}
