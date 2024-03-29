#ifndef _ISO_I2C_GPIO_H
#define _ISO_I2C_GPIO_H

#include "stm32f0xx.h"
#include <inttypes.h>

#define I2C_WR	0		/* 写控制bit */
#define I2C_RD	1		/* 读控制bit */

/* 定义I2C总线连接的GPIO端口, 用户只需要修改下面4行代码即可任意改变SCL和SDA的引脚 */
#define I2C_GPIO_PORT	GPIOB			/* GPIO端口 */
#define I2C_RCC_PORT 	RCC_AHBPeriph_GPIOB		/* GPIO端口时钟 */
#define I2C_SCL_PIN		GPIO_Pin_6			/* 连接到SCL时钟线的GPIO */
#define I2C_SDA_PIN		GPIO_Pin_7			/* 连接到SDA数据线的GPIO */

/* 定义读写SCL和SDA的宏，已增加代码的可移植性和可阅读性 */
#if 1	/* 条件编译： 1 选择GPIO的库函数实现IO读写 */
	#define I2C_SCL_1()  GPIO_SetBits(I2C_GPIO_PORT, I2C_SCL_PIN)		/* SCL = 1 */
	#define I2C_SCL_0()  GPIO_ResetBits(I2C_GPIO_PORT, I2C_SCL_PIN)		/* SCL = 0 */
	
	#define I2C_SDA_1()  GPIO_SetBits(I2C_GPIO_PORT, I2C_SDA_PIN)		/* SDA = 1 */
	#define I2C_SDA_0()  GPIO_ResetBits(I2C_GPIO_PORT, I2C_SDA_PIN)		/* SDA = 0 */
	
	#define I2C_SDA_READ()  GPIO_ReadInputDataBit(I2C_GPIO_PORT, I2C_SDA_PIN)	/* 读SDA口线状态 */
	#define I2C_SCL_READ()  GPIO_ReadInputDataBit(I2C_GPIO_PORT, I2C_SCL_PIN)	/* 读SCL口线状态 */
	
#else	/* 这个分支选择直接寄存器操作实现IO读写 */
    /*　注意：如下写法，在IAR最高级别优化时，会被编译器错误优化 */
	#define I2C_SCL_1()  I2C_GPIO_PORT->BSRR = I2C_SCL_PIN				/* SCL = 1 */
	#define I2C_SCL_0()  I2C_GPIO_PORT->BRR = I2C_SCL_PIN				/* SCL = 0 */
	
	#define I2C_SDA_1()  I2C_GPIO_PORT->BSRR = I2C_SDA_PIN				/* SDA = 1 */
	#define I2C_SDA_0()  I2C_GPIO_PORT->BRR = I2C_SDA_PIN				/* SDA = 0 */
	
	#define I2C_SDA_READ()  ((I2C_GPIO_PORT->IDR & I2C_SDA_PIN) != 0)	/* 读SDA口线状态 */
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
