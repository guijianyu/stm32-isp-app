#ifndef _ISO_I2C_GPIO_H
#define _ISO_I2C_GPIO_H

#include "stm32f0xx.h"
#include <inttypes.h>

#define I2C_WR	0		/* д����bit */
#define I2C_RD	1		/* ������bit */

/* ����I2C�������ӵ�GPIO�˿�, �û�ֻ��Ҫ�޸�����4�д��뼴������ı�SCL��SDA������ */
#define I2C_GPIO_PORT	GPIOB			/* GPIO�˿� */
#define I2C_RCC_PORT 	RCC_AHBPeriph_GPIOB		/* GPIO�˿�ʱ�� */
#define I2C_SCL_PIN		GPIO_Pin_6			/* ���ӵ�SCLʱ���ߵ�GPIO */
#define I2C_SDA_PIN		GPIO_Pin_7			/* ���ӵ�SDA�����ߵ�GPIO */

/* �����дSCL��SDA�ĺ꣬�����Ӵ���Ŀ���ֲ�ԺͿ��Ķ��� */
#if 1	/* �������룺 1 ѡ��GPIO�Ŀ⺯��ʵ��IO��д */
	#define I2C_SCL_1()  GPIO_SetBits(I2C_GPIO_PORT, I2C_SCL_PIN)		/* SCL = 1 */
	#define I2C_SCL_0()  GPIO_ResetBits(I2C_GPIO_PORT, I2C_SCL_PIN)		/* SCL = 0 */
	
	#define I2C_SDA_1()  GPIO_SetBits(I2C_GPIO_PORT, I2C_SDA_PIN)		/* SDA = 1 */
	#define I2C_SDA_0()  GPIO_ResetBits(I2C_GPIO_PORT, I2C_SDA_PIN)		/* SDA = 0 */
	
	#define I2C_SDA_READ()  GPIO_ReadInputDataBit(I2C_GPIO_PORT, I2C_SDA_PIN)	/* ��SDA����״̬ */
	#define I2C_SCL_READ()  GPIO_ReadInputDataBit(I2C_GPIO_PORT, I2C_SCL_PIN)	/* ��SCL����״̬ */
	
#else	/* �����֧ѡ��ֱ�ӼĴ�������ʵ��IO��д */
    /*��ע�⣺����д������IAR��߼����Ż�ʱ���ᱻ�����������Ż� */
	#define I2C_SCL_1()  I2C_GPIO_PORT->BSRR = I2C_SCL_PIN				/* SCL = 1 */
	#define I2C_SCL_0()  I2C_GPIO_PORT->BRR = I2C_SCL_PIN				/* SCL = 0 */
	
	#define I2C_SDA_1()  I2C_GPIO_PORT->BSRR = I2C_SDA_PIN				/* SDA = 1 */
	#define I2C_SDA_0()  I2C_GPIO_PORT->BRR = I2C_SDA_PIN				/* SDA = 0 */
	
	#define I2C_SDA_READ()  ((I2C_GPIO_PORT->IDR & I2C_SDA_PIN) != 0)	/* ��SDA����״̬ */
#endif

typedef enum {
	I2C_BUS_IDEL=0,
	I2C_BUS_BUSY,
	I2C_BUS_INVALID,
}E_I2C_BUS_STATE;

void I2C_Start(void);
void I2C_Stop(void);
void I2C_SendByte(uint8_t _ucByte);
uint8_t I2C_ReadByte(void);
uint8_t I2C_WaitAck(void);
void I2C_Ack(void);
void I2C_NAck(void);
E_I2C_BUS_STATE I2C_Probe(void);
void I2C_Bus_Free(void);
uint8_t I2C_CheckDevice(uint8_t _Address);
void I2C_GPIO_Config(void);

#endif
