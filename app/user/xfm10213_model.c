#include <stdio.h>

#include "iso_I2C_gpio.h"
#include "xfm10213.h"
#include "gpio.h"
#include "bsp_delay.h"
#include "xfm10213_model.h"
#include "led.h"

static __IO uint8_t xfm_rdy_flag = 0; /* xfm状态就绪标志 */


/*
*********************************************************************************************************
*	函 数 名: xfm_load
*	功能说明: 向xfm10213载入xfm10213.ldr，引导模块启动
*	形    参：无
*	返 回 值: 0 表示失败，1表示成功
*********************************************************************************************************
*/
uint8_t xfm_load(void)
{
	uint32_t i;
	const uint8_t *_pWriteBuf = ldr;
	uint32_t _usSize = sizeof(ldr);

	/* 第1步：发起I2C总线启动信号 */
	I2C_Start();
	
	/* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	I2C_SendByte(XFM_DEV_ADDR | I2C_WR); /* 此处是写指令 */
	
	/* 第3步：发送一个时钟，判断器件是否正确应答 */
	if (I2C_WaitAck() != 0)
	{
		/* 没有检测到xfm10213*/
		printf("\r\n 没有检测到xfm10213!\r\n");
		goto cmd_fail;
	}
	
	for (i = 0; i < _usSize; i++){
		//printf("\r\n %d", i);
		/* 第6步：开始写入数据 */
		I2C_SendByte(_pWriteBuf[i]);
	
		/* 第7步：发送ACK */
		if (I2C_WaitAck() != 0)
		{
			goto cmd_fail;	/* EEPROM器件无应答 */
		}
		
	}
	
	/* 命令执行成功，发送I2C总线停止信号 */
	I2C_Stop();
	printf("\r\n 引导完成!\r\n");
	return 1;

cmd_fail: /* 命令执行失败后，切记发送停止信号，避免影响I2C总线上其他设备 */
	/* 发送I2C总线停止信号 */
	I2C_Stop();
	printf("\r\n 引导失败!\r\n");
	return 0;
}

/*
*********************************************************************************************************
*	函 数 名: xfm_WriteReg
*	功能说明: 向xfm10213指定地址写入数据
*	形    参：_usAddress : 起始地址
*			 _value	    : 数据
*	返 回 值: 0 表示失败，1表示成功
*********************************************************************************************************
*/
uint8_t xfm_WriteReg(uint16_t _usAddress, uint16_t _value)
{
	uint16_t usAddr;
	usAddr = usAddr;
	usAddr = _usAddress;
	
	/* 第1步：发起I2C总线启动信号 */
	I2C_Start();
	
	/* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	I2C_SendByte(XFM_DEV_ADDR | I2C_WR); /* 此处是写指令 */
	
	/* 第3步：发送一个时钟，判断器件是否正确应答 */
	if (I2C_WaitAck() != 0){		
		goto cmd_fail;	
	}

	/* 第4步：发送寄存器高字节 */
	I2C_SendByte((uint8_t)(_usAddress>>8) & 0xFF);
	if (I2C_WaitAck() != 0){
		goto cmd_fail;	
	}
	
	/* 第5步：发送寄存器低字节 */
	I2C_SendByte((uint8_t)_usAddress & 0xFF);
	if (I2C_WaitAck() != 0){
		goto cmd_fail;	
	}
	
	/* 第6步：发送数据高字节 */
	I2C_SendByte((uint8_t)(_value>>8) & 0xFF);
	if (I2C_WaitAck() != 0){
		goto cmd_fail;	
	}
	
	/* 第7步：发送数据低字节 */
	I2C_SendByte((uint8_t)_value & 0xFF);
	if (I2C_WaitAck() != 0){	
		goto cmd_fail;	
	}
	/* 命令执行成功，发送I2C总线停止信号 */
	I2C_Stop();
	//printf("\r\n write %04X TO %04X OK!\r\n", _value, _usAddress);
	return 1;

cmd_fail: /* 命令执行失败后，切记发送停止信号，避免影响I2C总线上其他设备 */
	/* 发送I2C总线停止信号 */
	I2C_Stop();
	printf("\r\n write %04X TO %04X fail!\r\n", _value, _usAddress);
	return 0;
}

/*
*********************************************************************************************************
*	函 数 名: ee_ReadBytes
*	功能说明: 从xfm10213指定地址处开始读取若干数据
*	形    参：_usAddress : 起始地址, 
*	返 回 值: 0 表示失败，1表示成功
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
	/* 第1步：发起I2C总线启动信号 */
	I2C_Start();
	
	/* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	I2C_SendByte(XFM_DEV_ADDR | I2C_WR);	/* 此处是写指令 */
	
	/* 第3步：发送ACK */
	if (I2C_WaitAck() != 0)
	{	
		goto cmd_fail;	/* 无应答 */
	}

	/* 第4步：发送2字节地址*/
	I2C_SendByte((uint8_t)(_usAddress >> 8));
	
	/* 第5步：发送ACK */
	if (I2C_WaitAck() != 0)
	{	
		goto cmd_fail;	/* 无应答 */
	}

	/* 第6步：发送2字节地址*/
	I2C_SendByte((uint8_t)_usAddress);
	
	/* 第7步：发送ACK */
	if (I2C_WaitAck() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}
	
	/* 第8步：重新启动I2C总线。前面的代码的目的向EEPROM传送地址，下面开始读取数据 */
	I2C_Start();
	
	/* 第9步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	I2C_SendByte(XFM_DEV_ADDR | I2C_RD);	/* 此处是读指令 */
	
	/* 第10步：发送ACK */
	if (I2C_WaitAck() != 0)
	{		
		goto cmd_fail;	/* EEPROM器件无应答 */
	}	
	
	/* 第11步：循环读取数据 */
	for (i = 0; i < 2; i++)
	{
		value <<= 8;
		value += I2C_ReadByte();	/* 读1个字节 */
		/* 每读完1个字节后，需要发送Ack， 最后一个字节不需要Ack，发Nack */
		if (i != 1)
		{
			I2C_Ack();	/* 中间字节读完后，CPU产生ACK信号(驱动SDA = 0) */
		}
		else
		{
			I2C_NAck();	/* 最后1个字节读完后，CPU产生NACK信号(驱动SDA = 1) */
		}
	}
	/* 发送I2C总线停止信号 */
	I2C_Stop();
	*reg_value = value;
	//printf("\r\n read %04X from %04X OK!\r\n", value, _usAddress);
	return 1;	/* 执行成功 */

cmd_fail: /* 命令执行失败后，切记发送停止信号，避免影响I2C总线上其他设备 */
	/* 发送I2C总线停止信号 */
	I2C_Stop();
	printf("\r\n read from %04X fail!\r\n", _usAddress);
	return 0;
}

/*
*********************************************************************************************************
*	函 数 名: xfm_irq_process
*	功能说明: xfm中断处理函数
*	形    参：无
*	返 回 值: 0 表示失败，1表示成功
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
			case xfm_RDY:  /* 系统就绪状态 */
				xfm_rdy_flag = 1;
				break;
			case xfm_voice_start:
				break;
			case xfm_voice_end:
				break;
			case xfm_voice_wake:
				if(g_648_work_state == SLEEP_STATE) /* 当前睡眠状态 */
				{
					LED_Open(LED1);		/* 开启绿灯 */
					LED_Close(LED2);    /* 关闭红灯 */						
					Stm32_Set_Gpio_Ouput(GPIO_NUM_1,HIGH_LEVEL);  /* Mstar648使用独特的音乐启动 */
					Stm32_Wake_Mstar648_Chip_Enable();  		  /* 使能唤醒648 */				
				}
				break;
				
			default: /* 其他值表示错误状态，不应该出现，出现则需要复位xfm10213 */
				Stm32_Reset_XFM10213_Chip_Trigger();
				break;				
		}
	}

	return 1;
}

/*
*********************************************************************************************************
*	函 数 名: xfm_init
*	功能说明: xfm初始化函数
*	形    参：无
*	返 回 值: 无
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
			printf("\r\n xfm_init 中load失败!\r\n");
			continue;
		}
		else
		{
			while(xfm_rdy_flag != 1);  /* 一直等到xfm10213芯片初始化就绪 */
				
			//__disable_irq();  /*关闭中断*/
			xfm_rdy_flag = 0;
			//__enable_irq();	  /*开启中断*/
			
			bsp_DelayMS(20);
			xfm_WriteReg(XFM_CHANNEL_REG, XFM_4CHANNEL_OUTPUT); /*通道设置寄存器，设置为4通道输出*/
			bsp_DelayMS(20);
			xfm_WriteReg(XFM_MODE_REG, XFM_MIXED_RUN_MODE);		/*模式寄存器，设置为混合运行模式*/
			bsp_DelayMS(50);
			printf("\r\n xfm run !\r\n");
			return;
		}	
	}
}
