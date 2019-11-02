#include <stdio.h>

#include "iso_I2C_gpio.h"
#include "xfm10213.h"
#include "gpio.h"
#include "bsp_delay.h"
#include "xfm10213_model.h"
#include "led.h"

static __IO uint8_t xfm_rdy_flag = 0; /* xfm״̬������־ */


/*
*********************************************************************************************************
*	�� �� ��: xfm_load
*	����˵��: ��xfm10213����xfm10213.ldr������ģ������
*	��    �Σ���
*	�� �� ֵ: 0 ��ʾʧ�ܣ�1��ʾ�ɹ�
*********************************************************************************************************
*/
uint8_t xfm_load(void)
{
	uint32_t i;
	const uint8_t *_pWriteBuf = ldr;
	uint32_t _usSize = sizeof(ldr);

	/* ��1��������I2C���������ź� */
	I2C_Start();
	
	/* ��2������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
	I2C_SendByte(XFM_DEV_ADDR | I2C_WR); /* �˴���дָ�� */
	
	/* ��3��������һ��ʱ�ӣ��ж������Ƿ���ȷӦ�� */
	if (I2C_WaitAck() != 0)
	{
		/* û�м�⵽xfm10213*/
		printf("\r\n û�м�⵽xfm10213!\r\n");
		goto cmd_fail;
	}
	
	for (i = 0; i < _usSize; i++){
		//printf("\r\n %d", i);
		/* ��6������ʼд������ */
		I2C_SendByte(_pWriteBuf[i]);
	
		/* ��7��������ACK */
		if (I2C_WaitAck() != 0)
		{
			goto cmd_fail;	/* EEPROM������Ӧ�� */
		}
		
	}
	
	/* ����ִ�гɹ�������I2C����ֹͣ�ź� */
	I2C_Stop();
	printf("\r\n �������!\r\n");
	return 1;

cmd_fail: /* ����ִ��ʧ�ܺ��мǷ���ֹͣ�źţ�����Ӱ��I2C�����������豸 */
	/* ����I2C����ֹͣ�ź� */
	I2C_Stop();
	printf("\r\n ����ʧ��!\r\n");
	return 0;
}

/*
*********************************************************************************************************
*	�� �� ��: xfm_WriteReg
*	����˵��: ��xfm10213ָ����ַд������
*	��    �Σ�_usAddress : ��ʼ��ַ
*			 _value	    : ����
*	�� �� ֵ: 0 ��ʾʧ�ܣ�1��ʾ�ɹ�
*********************************************************************************************************
*/
uint8_t xfm_WriteReg(uint16_t _usAddress, uint16_t _value)
{
	uint16_t usAddr;
	usAddr = usAddr;
	usAddr = _usAddress;
	
	/* ��1��������I2C���������ź� */
	I2C_Start();
	
	/* ��2������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
	I2C_SendByte(XFM_DEV_ADDR | I2C_WR); /* �˴���дָ�� */
	
	/* ��3��������һ��ʱ�ӣ��ж������Ƿ���ȷӦ�� */
	if (I2C_WaitAck() != 0){		
		goto cmd_fail;	
	}

	/* ��4�������ͼĴ������ֽ� */
	I2C_SendByte((uint8_t)(_usAddress>>8) & 0xFF);
	if (I2C_WaitAck() != 0){
		goto cmd_fail;	
	}
	
	/* ��5�������ͼĴ������ֽ� */
	I2C_SendByte((uint8_t)_usAddress & 0xFF);
	if (I2C_WaitAck() != 0){
		goto cmd_fail;	
	}
	
	/* ��6�����������ݸ��ֽ� */
	I2C_SendByte((uint8_t)(_value>>8) & 0xFF);
	if (I2C_WaitAck() != 0){
		goto cmd_fail;	
	}
	
	/* ��7�����������ݵ��ֽ� */
	I2C_SendByte((uint8_t)_value & 0xFF);
	if (I2C_WaitAck() != 0){	
		goto cmd_fail;	
	}
	/* ����ִ�гɹ�������I2C����ֹͣ�ź� */
	I2C_Stop();
	//printf("\r\n write %04X TO %04X OK!\r\n", _value, _usAddress);
	return 1;

cmd_fail: /* ����ִ��ʧ�ܺ��мǷ���ֹͣ�źţ�����Ӱ��I2C�����������豸 */
	/* ����I2C����ֹͣ�ź� */
	I2C_Stop();
	printf("\r\n write %04X TO %04X fail!\r\n", _value, _usAddress);
	return 0;
}

/*
*********************************************************************************************************
*	�� �� ��: ee_ReadBytes
*	����˵��: ��xfm10213ָ����ַ����ʼ��ȡ��������
*	��    �Σ�_usAddress : ��ʼ��ַ, 
*	�� �� ֵ: 0 ��ʾʧ�ܣ�1��ʾ�ɹ�
*********************************************************************************************************
*/
int8_t xfm_ReadReg(uint16_t _usAddress, uint16_t* reg_value)
{
	uint16_t i;
	uint16_t value = 0;
	if(reg_value == NULL)
	{
		return 0;
	}
	/* ��1��������I2C���������ź� */
	I2C_Start();
	
	/* ��2������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
	I2C_SendByte(XFM_DEV_ADDR | I2C_WR);	/* �˴���дָ�� */
	
	/* ��3��������ACK */
	if (I2C_WaitAck() != 0)
	{	
		goto cmd_fail;	/* ��Ӧ�� */
	}

	/* ��4��������2�ֽڵ�ַ*/
	I2C_SendByte((uint8_t)(_usAddress >> 8));
	
	/* ��5��������ACK */
	if (I2C_WaitAck() != 0)
	{	
		goto cmd_fail;	/* ��Ӧ�� */
	}

	/* ��6��������2�ֽڵ�ַ*/
	I2C_SendByte((uint8_t)_usAddress);
	
	/* ��7��������ACK */
	if (I2C_WaitAck() != 0)
	{
		goto cmd_fail;	/* EEPROM������Ӧ�� */
	}
	
	/* ��8������������I2C���ߡ�ǰ��Ĵ����Ŀ����EEPROM���͵�ַ�����濪ʼ��ȡ���� */
	I2C_Start();
	
	/* ��9������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
	I2C_SendByte(XFM_DEV_ADDR | I2C_RD);	/* �˴��Ƕ�ָ�� */
	
	/* ��10��������ACK */
	if (I2C_WaitAck() != 0)
	{		
		goto cmd_fail;	/* EEPROM������Ӧ�� */
	}	
	
	/* ��11����ѭ����ȡ���� */
	for (i = 0; i < 2; i++)
	{
		value <<= 8;
		value += I2C_ReadByte();	/* ��1���ֽ� */
		/* ÿ����1���ֽں���Ҫ����Ack�� ���һ���ֽڲ���ҪAck����Nack */
		if (i != 1)
		{
			I2C_Ack();	/* �м��ֽڶ����CPU����ACK�ź�(����SDA = 0) */
		}
		else
		{
			I2C_NAck();	/* ���1���ֽڶ����CPU����NACK�ź�(����SDA = 1) */
		}
	}
	/* ����I2C����ֹͣ�ź� */
	I2C_Stop();
	*reg_value = value;
	//printf("\r\n read %04X from %04X OK!\r\n", value, _usAddress);
	return 1;	/* ִ�гɹ� */

cmd_fail: /* ����ִ��ʧ�ܺ��мǷ���ֹͣ�źţ�����Ӱ��I2C�����������豸 */
	/* ����I2C����ֹͣ�ź� */
	I2C_Stop();
	printf("\r\n read from %04X fail!\r\n", _usAddress);
	return 0;
}

/*
*********************************************************************************************************
*	�� �� ��: xfm_irq_process
*	����˵��: xfm�жϴ�����
*	��    �Σ���
*	�� �� ֵ: 0 ��ʾʧ�ܣ�1��ʾ�ɹ�
*********************************************************************************************************
*/
int8_t xfm_irq_process(void)
{
	uint16_t read_value = 0xFFFF;
	if(xfm_ReadReg(XFM_SYS_STATE_REG,&read_value) == 0)
	{
		return 0;
	}	
	else
	{
		//printf("whh %s XFM_SYS_STATE_REG =%#x\r\n",__FUNCTION__,read_value);
		switch(read_value)
		{
			case xfm_sys_init_stat:
				break;
			case xfm_RDY:  /* ϵͳ����״̬ */
				xfm_rdy_flag = 1;
				break;
			case xfm_voice_start:
				break;
			case xfm_voice_end:
				break;
			case xfm_voice_wake:
				if(g_648_work_state == SLEEP_STATE) /* ��ǰ˯��״̬ */
				{
					LED_Open(LED1);		/* �����̵� */
					LED_Close(LED2);    /* �رպ�� */						
					Stm32_Set_Gpio_Ouput(GPIO_NUM_1,HIGH_LEVEL);  /* Mstar648ʹ�ö��ص��������� */
					Stm32_Wake_Mstar648_Chip_Enable();  		  /* ʹ�ܻ���648 */				
				}
				break;
				
			default: /* ����ֵ��ʾ����״̬����Ӧ�ó��֣���������Ҫ��λxfm10213 */
				Stm32_Reset_XFM10213_Chip_Trigger();
				break;				
		}
	}

	return 1;
}

/*
*********************************************************************************************************
*	�� �� ��: xfm_init
*	����˵��: xfm��ʼ������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void xfm_init(void)
{
	while(1)
	{
		if(I2C_CheckDevice(XFM_DEV_ADDR) == 0)
		{
			printf("XFM device I2C bus OK! \r\n");			
			break;
		}
		else 
		{		
			printf("XFM device I2C bus error! \r\n");
			printf("SDA vol=%d \r\n",I2C_SDA_READ());
			printf("SCL vol=%d \r\n",I2C_SCL_READ());
		}
	}
	while(1)
	{
		if(xfm_load() == 0)
		{
			printf("\r\n xfm_init ��loadʧ��!\r\n");
			continue;
		}
		else
		{
			while(xfm_rdy_flag != 1);  /* һֱ�ȵ�xfm10213оƬ��ʼ������ */
				
			//__disable_irq();  /*�ر��ж�*/
			xfm_rdy_flag = 0;
			//__enable_irq();	  /*�����ж�*/
			
			bsp_DelayMS(20);
			xfm_WriteReg(XFM_CHANNEL_REG, XFM_4CHANNEL_OUTPUT); /*ͨ�����üĴ���������Ϊ4ͨ�����*/
			bsp_DelayMS(20);
			xfm_WriteReg(XFM_MODE_REG, XFM_MIXED_RUN_MODE);		/*ģʽ�Ĵ���������Ϊ�������ģʽ*/
			bsp_DelayMS(50);
			printf("\r\n xfm run !\r\n");
			return;
		}	
	}
}
