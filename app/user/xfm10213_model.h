#ifndef __XFM10213_MODEL_H
#define	__XFM10213_MODEL_H

#include "stm32f0xx.h"

#define XFM_DEV_ADDR			0x7C			/* XFM10213���豸��ַ */

/*�Ĵ�������*/
#define XFM_COMMUN_STATE_REG	0x0101			/*ͨ��״̬�Ĵ���(0x0101)*/
#define XFM_SYS_STATE_REG		0x0102			/*ϵͳ״̬�Ĵ���(0x0102)*/
#define XFM_MODE_REG			0x0110			/*ģʽ���üĴ���*/
#define XFM_CHANNEL_REG			0x0111			/*ͨ�����üĴ���*/

/*�������ݶ���*/
#define XFM_MIXED_RUN_MODE 		0x0004			/*�������ģʽ,֧�ֽ������桢�������沢������*/
#define XFM_4CHANNEL_OUTPUT		0x5656			/*��ͨ���������������������� 1����Ƶ���ݣ��������н�������ʱ��Ч�������������*/

typedef enum
{
	xfm_RDY = 0x0000,
	xfm_voice_start = 0x0001,
	xfm_voice_end = 0x0002,
	xfm_voice_wake = 0x0003,
	xfm_sys_init_stat = 0xFFFF,
}xfm_sys_state_e;

uint8_t xfm_load(void);
uint8_t xfm_WriteReg(uint16_t _usAddress, uint16_t _value);
int8_t xfm_ReadReg(uint16_t _usAddress, uint16_t* reg_value);
int8_t xfm_irq_process(void);
void xfm_init(void);

#endif /*__XFM10213_MODEL_H*/
