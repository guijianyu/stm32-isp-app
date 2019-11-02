#include "stepmotor.h"
#include "err.h"
#include "types.h"

__IO uint32_t BeatCycle = 0x00; 
__IO Rotate_t rotate;	//转向
__IO uint32_t beats;	//总节拍数
__IO uint32_t beatCount;	//当前节拍
extern __IO uint8_t warning;

void StepMotor_Config(void)
{
	/*
		AIN1~PA12, AIN2~PA13, BIN2~PA14, BIN1~PA15
	*/
	GPIO_InitTypeDef GPIO_InitStruct_Rotate;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_InitStruct_Rotate.GPIO_Pin = PIN_A | PIN_AN | PIN_B | PIN_BN;
	GPIO_InitStruct_Rotate.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct_Rotate.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct_Rotate.GPIO_Speed =GPIO_Speed_Level_3;
	GPIO_Init(GPIOA, &GPIO_InitStruct_Rotate);
	GPIO_SetBits(GPIOA, PIN_A | PIN_AN | PIN_B | PIN_BN);

	/*
		M_SLEEP~PB3
	*/
	GPIO_InitTypeDef GPIO_InitStruct_Sleep;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	GPIO_InitStruct_Sleep.GPIO_Pin = PIN_SLEEP;
	GPIO_InitStruct_Sleep.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct_Sleep.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct_Sleep.GPIO_Speed =GPIO_Speed_Level_3;
	GPIO_Init(GPIOB, &GPIO_InitStruct_Sleep);
	GPIO_SetBits(GPIOB, PIN_SLEEP);

	/*
		MFAULT~PB4, MOTOR_SENSOR~PB5
	*/
	GPIO_InitTypeDef GPIO_InitStruct_Fault;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	GPIO_InitStruct_Fault.GPIO_Pin = PIN_FAULT;
	GPIO_InitStruct_Fault.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct_Fault.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct_Fault);

	GPIO_InitTypeDef GPIO_InitStruct_Sensor;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	GPIO_InitStruct_Sensor.GPIO_Pin = PIN_SENSOR;
	GPIO_InitStruct_Sensor.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct_Sensor.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct_Sensor);
}

//设置A,AN管脚，控制步进电机A线圈磁场相位
static void PhaseA(Phase_t ph)
{
	switch(ph){
	case N:
		GPIO_ResetBits(GPIOA, PIN_A);
		GPIO_SetBits(GPIOA, PIN_AN);
		break;
	case P:
		GPIO_SetBits(GPIOA, PIN_A);
		GPIO_ResetBits(GPIOA, PIN_AN);
		break;
	case Z:
		GPIO_SetBits(GPIOA, PIN_A | PIN_AN);
		break;
	}
}

//设置B,BN管脚，控制步进电机B线圈磁场相位
static void PhaseB(Phase_t ph)
{
	switch(ph){
	case N:
		GPIO_ResetBits(GPIOA, PIN_B);
		GPIO_SetBits(GPIOA, PIN_BN);
		break;
	case P:
		GPIO_SetBits(GPIOA, PIN_B);
		GPIO_ResetBits(GPIOA, PIN_BN);
		break;
	case Z:
		GPIO_SetBits(GPIOA, PIN_B | PIN_BN);
		break;
	}
}

////8拍正转：A - AB - B - ANB - AN - ANBN - BN - ABN
//static const Phase_t CW[PHASES][POLARIY] = {{P,Z}, {P,P}, {Z,P}, {N,P}, {N,Z}, {N,N}, {Z,N}, {P,N}};

////8拍反转：A  - ABN - BN - ANBN - AN - ANB - B - AB
//static const Phase_t CCW[PHASES][POLARIY] = {{P,Z}, {P,N}, {Z,N}, {N,N}, {N,Z}, {N,P}, {Z,P}, {P,P}};

//4拍正转：ANB - AB - ABN - ANBN
static const Phase_t CW[PHASES][POLARIY] = {{N,P}, {P,P}, {P,N}, {N,N}};

//4拍反转：ANBN - ABN - AB - ANB
static const Phase_t CCW[PHASES][POLARIY] = {{N,N}, {P,N}, {P,P}, {N,P}};
void beat(void)
{
	if(SM_Limit() && (rotate == CounterClockWise)){
		warning |= MOTOR_LIMIT;
		goto err;
	}
	//驱动芯片出错
	if(SM_Fault()){
		warning |= MOTOR_FAULT;
		goto err;
	}

	switch(rotate){
	case ClockWise:
		PhaseA(CW[beatCount%PHASES][0]);
		PhaseB(CW[beatCount%PHASES][1]);
		break;
	case CounterClockWise:
		PhaseA(CCW[beatCount%PHASES][0]);
		PhaseB(CCW[beatCount%PHASES][1]);
		break;
	default:
		break;
	}

	if(beatCount < beats){
		beatCount++;
		BeatCycle = BEAT_TICKS;
	}else{
		PhaseA(Z);
		PhaseB(Z);
	}
	return;

err:
	PhaseA(Z);
	PhaseB(Z);
	return;
}

//SM_Rotate不再使用
uint8_t SM_Rotate(Rotate_t rt, uint32_t bts)
{
	uint8_t ret = 0, i, j;
	
	switch(rt){
	case ClockWise:
		for(i = 0; i < (bts / PHASES); i++){
			for(j = 0; j < PHASES; j++){
				PhaseA(CW[j][0]);
				PhaseB(CW[j][1]);
			}
		}
		for(i = 0; i < (bts % PHASES); i++){
			PhaseA(CW[i][0]);
			PhaseB(CW[i][1]);
		}
		break;
	case CounterClockWise:
		for(i = 0; i < (bts / PHASES); i++){
			for(j = 0; j < PHASES; j++){
				PhaseA(CCW[j][0]);
				PhaseB(CCW[j][1]);
			}
		}
		for(i = 0; i < (bts % PHASES); i++){
			PhaseA(CCW[i][0]);
			PhaseB(CCW[i][1]);
		}
		break;
	default:
		ret = EARG;
		break;
	}
	return ret;
}

uint8_t SM_Fault(void)
{
	return !GPIO_ReadInputDataBit(GPIOB, PIN_FAULT);
}

uint8_t SM_Limit(void)
{
	return GPIO_ReadInputDataBit(GPIOB, PIN_SENSOR);
}

void SM_Sleep(void)
{
	GPIO_ResetBits(GPIOB, PIN_SLEEP);
}

void SM_Wake(void)
{
	GPIO_SetBits(GPIOB, PIN_SLEEP);
}

