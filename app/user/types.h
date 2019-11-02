#ifndef __TYPES_H
#define	__TYPES_H
#include "stm32f0xx.h"

#define BUFFERSIZE   0x08	//单个通信协议帧大小
#define RING_SIZE	32	//协议帧缓冲个数

typedef enum
{
	FALSE =0,
	TRUE =!FALSE
} bool;

typedef enum{
	CMD_PWM = 0x01,	//风扇pwm设置
	CMD_FUNSPEED,	//0x02，风扇转速获取
	CMD_STEPMOTOR,	//0x03，步进电机控制
	CMD_SLEEP,	//0x04， 步进电机睡眠
	CMD_GET_TEMPE_VALUE,	//0x05，获取温度
	CMD_GET_LIGHT_INTENSITY,	//0x06，获取光强
	CMD_GET_ENV,	//0x07，获取环境变量
	CMD_SET_ENV,	//0x08，设置环境变量
	CMD_LIMIT,
	CMD_XFM,
}cmd_t;

//stm32与mstar串口通信协议帧
struct msg{
	cmd_t cmd:8;	//命令
	uint8_t sn;		//命令序列号
	uint8_t data[BUFFERSIZE - 3];	//数据
	uint8_t check;	//简单校验：cmd,sn,data[5]求和，取低八位
};

struct ringBuffer{
	uint8_t head;		//缓冲区头部位置
	uint8_t tail;	//缓冲区尾部位置
	struct msg buf[RING_SIZE];	//缓冲区数组
};

#define KEY_UPGRADE	0x31

#define MOTOR_LIMIT 0x01	//光机到达限位
#define MOTOR_FAULT	0x02	//光机驱动芯片故障
#define TEMP_LIMIT	0x04	//温度告警


#endif

