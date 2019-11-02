#ifndef __BSP_KEY_H
#define	__BSP_KEY_H

#include "stm32f0xx.h"

/*
	�����еĻ���Դ���������¼�����
*/
#define WAKE_SOURCE_COUNT    1	    /*����Դ�ĸ���*/ 	

/*
	�����˲�ʱ��10ms, ��λ10ms��
	ֻ��������⵽50ms״̬�������Ϊ��Ч����������Ͱ��������¼�
	��ʹ������·����Ӳ���˲������˲�����Ҳ���Ա�֤�ɿ��ؼ�⵽�����¼�
*/
#define KEY_FILTER_TIME   1

/* ����ID, ��Ҫ����bsp_KeyState()��������ڲ��� */
typedef enum
{
	KID_K1 = 0,
}KEY_ID_E;

/*
	�����ֵ����, ���밴���´���ʱÿ�����İ��¡�����ͳ����¼�

	�Ƽ�ʹ��enum, ����#define��ԭ��
	(1) ����������ֵ,�������˳��ʹ���뿴���������
	(2) �������ɰ����Ǳ����ֵ�ظ���
*/
typedef enum
{
	KEY_NONE = 0,					/* 0 ��ʾ�����¼� */

	KEY_1_DOWN,						/* 1������ */
	KEY_1_UP,						/* 1������ */
}KEY_ENUM;


/* ����FIFO�õ����� */
#define KEY_FIFO_SIZE	10
typedef struct
{
	uint8_t Buf[KEY_FIFO_SIZE];		/* ��ֵ������ */
	uint8_t Read;					/* ��������ָ��1 */
	uint8_t Write;					/* ������дָ�� */
}KEY_FIFO_T;

/*
	ÿ��������Ӧ1��ȫ�ֵĽṹ�������
*/
typedef struct
{
	/* ������һ������ָ�룬ָ���жϰ����ַ��µĺ��� */
	uint8_t (*IsKeyDownFunc)(void); /* �������µ��жϺ���,1��ʾ���� */

	uint8_t  Count;			/* �˲��������� */
	uint8_t  State;			/* ������ǰ״̬�����»��ǵ��� */
}KEY_T;

/*���еĻ����¼���Ϊ�ǰ���ʱ�䣬���Ѱ���ɨ��*/
void bsp_key_scan(void);
void bsp_key_event_process(void);

/* ���ⲿ���õĺ������� */
void bsp_InitKey(void);
void bsp_PutKey(uint8_t _KeyCode);
uint8_t bsp_GetKey(void);
uint8_t bsp_GetKeyState(KEY_ID_E _ucKeyID);
void bsp_ClearKey(void);

#endif  //__BSP_KEY_H
