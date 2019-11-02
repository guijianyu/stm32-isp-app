#include "fun.h"
#include "err.h"
#include "uart.h"
#include "adc.h"
#include <string.h>
#include "dlp_control.h"

#define DELAY_FUN	(60*SYSTICK_PERIOD)	//1min	

uint16_t TimerPeriod = 0;
__IO uint32_t TIM2CHFreq[2] = {0};
extern __IO uint8_t warning;
__IO uint32_t funtime = 0;

/*
环境温度上升到30摄氏度以上后，即使环境温度下降到30以下，ledr～风速控制策略也会遵循strategy_env2_on[],strategy_env2_off[]
ledr下降到38以下，遵循strategy_env1_on[],strategy_env1_off[]
*/
#define FUN_MAX_SPEED	4500
//环境温度0-30摄氏度 ~ ledR温度转速策略,温度升高和降低过程中关键节点
static const struct temp_rpm strategy_env1_on[] = {{0, 2300}, {40, 2700}, {53, 3000}, {57, 3600}, {60, FUN_MAX_SPEED}};
static const struct temp_rpm strategy_env1_off[] = {{0, 2300}, {35, 2700}, {50, 3000}, {53, 3600}, {57, FUN_MAX_SPEED}};
bool env1_node[5];	//记录当前温度处于下降过程还是上升过程

//环境温度30-40摄氏度 ~ ledR温度转速策略,温度升高和降低过程中关键节点
static struct temp_rpm strategy_env2_on[] = {{0, 2700}, {40, 3600}, {57, FUN_MAX_SPEED}};
static struct temp_rpm strategy_env2_off[] = {{0, 2700}, {38, 3600}, {53, FUN_MAX_SPEED}};
bool env2_node[3];	//记录当前温度处于下降过程还是上升过程

void Fun_PWM_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructurePWM;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	uint16_t ChannelPulse = 0;

	/* GPIOA Clocks enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	
	GPIO_InitStructurePWM.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11;
	GPIO_InitStructurePWM.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructurePWM.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructurePWM.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructurePWM.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOA, &GPIO_InitStructurePWM);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_2);

	/* Compute the value to be set in ARR regiter to generate signal frequency at 25 Khz */
	TimerPeriod = (SystemCoreClock / FUN_PWM_FREQ ) - 1;

	/* Compute CCR1 value to generate a duty cycle at 90% for channel 1 */
	ChannelPulse = (uint16_t) (((uint32_t) DEFAULT_DUTY * (TimerPeriod - 1)) / 100);
	
	/* TIM1 clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);
	
	/* Time Base configuration */
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	
	/* Channel 1 Configuration in PWM out compare mode of TIM1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	
	TIM_OCInitStructure.TIM_Pulse = ChannelPulse;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);   
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM1, ENABLE);
	
	/* TIM1 counter enable */
	TIM_Cmd(TIM1, ENABLE);
  
	/* TIM1 Main Output Enable */
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	
	memset(env1_node, 0, sizeof(env1_node));
	memset(env2_node, 0, sizeof(env2_node));
}

void Fun_Speed_Config(void)
{
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* TIM2 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	/* GPIOB clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	
	/* TIM2 channel 3 pin (PB10,PB11) configuration */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* Connect TIM pins to AF2 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_2);

	/* Time Base configuration */
	TIM_TimeBaseStructure.TIM_Prescaler = TIM2_PRESCALER - 1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	//
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	/* TIM1 configuration: Input Capture mode ---------------------
	   The external signal is connected to TIM2 CH1 pin (PA0,PA1)  
	   The Rising edge is used as active edge,
	   The TIM1 CCR2 is used to compute the frequency value 
	------------------------------------------------------------ */
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;
	
	TIM_ICInit(TIM2, &TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
	
	/* TIM enable counter */
	TIM_Cmd(TIM2, ENABLE);
	
	/* Enable the CC1,CC2 Interrupt Request */
	TIM_ITConfig(TIM2, TIM_IT_CC3, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_CC4, ENABLE);
	
	/* Enable the TIM1 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

uint8_t PWM_Set_Pulse(Fun_t ft, uint8_t duty)
{
	//static uint8_t pwm_flag = 0; //1：表示当前pwm为0或者100，  0：表示使用TIM输出pwm
	uint16_t pulse;	
	uint8_t ret = 0;
	
	if(duty > MAX_DUTY)
	{	
		duty = MAX_DUTY;
	}
	
	#if 0
	if (duty == 0)
	{		
		pwm_flag = 1;
		TIM_Cmd(TIM1, DISABLE);		/* 关闭PWM输出 */
		if(ft == Fun1)
		{
			bsp_ConfigGpioOut(GPIOA, GPIO_Pin_8);	/* 配置GPIO为推挽输出 */	
			GPIO_WriteBit(GPIOA, GPIO_Pin_8, Bit_RESET);	/* PWM = 0 */				
		}
		else if (ft == Fun2)
		{
			bsp_ConfigGpioOut(GPIOA, GPIO_Pin_11);	/* 配置GPIO为推挽输出 */	
			GPIO_WriteBit(GPIOA, GPIO_Pin_11, Bit_RESET);	/* PWM = 0 */	
		}			
		return ret;
	}
	else if (duty == 100)
	{		
		pwm_flag = 1;
		TIM_Cmd(TIM1, DISABLE);		/* 关闭PWM输出 */
		if(ft == Fun1)
		{
			bsp_ConfigGpioOut(GPIOA, GPIO_Pin_8);	/* 配置GPIO为推挽输出 */	
			GPIO_WriteBit(GPIOA, GPIO_Pin_8, Bit_SET);	/* PWM = 1 */				
		}
		else if (ft == Fun2)
		{
			bsp_ConfigGpioOut(GPIOA, GPIO_Pin_11);	/* 配置GPIO为推挽输出 */	
			GPIO_WriteBit(GPIOA, GPIO_Pin_11, Bit_SET);	/* PWM = 1 */	
		}					
		return ret;
	}	
	else 
	#endif 
	
	{
		#if 0
		if(pwm_flag == 1)
		{
			Fun_PWM_Config();
			pwm_flag = 0; //清除标志
		}
		#endif 
		
		pulse= (uint16_t) (((uint32_t) duty * (TimerPeriod - 1)) / 100);
		switch(ft){
		case Fun1:
			TIM_SetCompare1(TIM1, pulse);
			break;
		case Fun2:
			TIM_SetCompare4(TIM1, pulse);	
			break;
		default:
			ret = EARG;
			break;
		}
		return ret;
	}
}

uint32_t get_rotate_speed(Fun_t ft)
{
	return TIM2CHFreq[ft] / 2 * 60;
}

/*
	fun_tick_start	开始计时
	fun_tick		计时中
	fun_tick_finish	计时是否结束
*/
static void fun_tick_start(uint32_t delay)
{
	funtime = delay;
}

void inline fun_tick(void)
{
	if (funtime != 0x00){
		funtime--;
	}
}

static uint32_t fun_tick_finish()
{
	return !funtime;
}

static const struct rpm_duty rd[] = {{900, 30}, {1250, 40}, {1900, 50}, {2600, 60}, {3200, 70}, {3800, 80}, {4200, 90}, {4500, 100}};
static uint8_t linear(uint16_t rpm){
	int i, size = ARRAY_SIZE(rd);
	if(rpm <= rd[0].rpm)
		return rd[0].duty;

	if(rpm >= rd[size - 1].rpm)
		return rd[size - 1].duty;
	
	for(i = 0; i < (size - 1); i++){
		if((rpm > rd[i].rpm) && (rpm <= rd[i+1].rpm))
			break;
	}
	return ((float)rd[i+1].duty - (float)rd[i].duty)/(rd[i+1].rpm - rd[i].rpm)*(rpm - rd[i].rpm) + rd[i].duty;
}

void regulate_fun(uint16_t rpm){
	uint8_t dt = linear(rpm);

	PWM_Set_Pulse(Fun1, dt);
	PWM_Set_Pulse(Fun2, dt);
}

/*
 *根据温度调节风扇转速
*/
void temp_rpm_regulate(uint8_t env, uint8_t ledr)
{
	uint16_t rpm;
	struct msg res;
	memset(&res, 0xFF, BUFFERSIZE);
	
	if(env < TEMP_ENV_NODE0){
		if(ledr <= strategy_env1_off[1].temp){
			rpm = strategy_env1_off[0].rpm;
			env1_node[1] = FALSE;
			env1_node[2] = FALSE;
			env1_node[3] = FALSE;
			env1_node[4] = FALSE;
		}else if(ledr <= strategy_env1_on[1].temp){
			if(env1_node[1])
				rpm = strategy_env1_off[1].rpm;
			else
				rpm = strategy_env1_on[0].rpm;

			env1_node[2] = FALSE;
			env1_node[3] = FALSE;
			env1_node[4] = FALSE;
 		}else if(ledr <= strategy_env1_off[2].temp){
			rpm = strategy_env1_on[1].rpm;
			env1_node[1] = TRUE;
			env1_node[2] = FALSE;
			env1_node[3] = FALSE;
			env1_node[4] = FALSE;
		}else if(ledr <= strategy_env1_on[2].temp){
			if(env1_node[2])
				rpm = strategy_env1_off[2].rpm;
			else
				rpm = strategy_env1_on[1].rpm;
			
			env1_node[1] = TRUE;
			env1_node[3] = FALSE;
			env1_node[4] = FALSE;
		}else if(ledr <= strategy_env1_on[3].temp){
			if(env1_node[3])
				rpm = strategy_env1_off[3].rpm;
			else
				rpm = strategy_env1_on[2].rpm;
			
			env1_node[1] = TRUE;
			env1_node[2] = TRUE;
			env1_node[4] = FALSE;
		}else if(ledr <= strategy_env1_on[4].temp){
			if(env1_node[4])
				rpm = strategy_env1_off[4].rpm;
			else
				rpm = strategy_env1_on[3].rpm;
			
			env1_node[1] = TRUE;
			env1_node[2] = TRUE;
			env1_node[3] = TRUE;
		}else{
			//出现温度预警事件
			if(fun_tick_finish()){
				warning |= TEMP_LIMIT;
				fun_tick_start(DELAY_FUN);
			}
			
			rpm = strategy_env1_on[4].rpm;
			env1_node[1] = TRUE;
			env1_node[2] = TRUE;
			env1_node[3] = TRUE;
			env1_node[4] = TRUE;
		}
	}else if(env < TEMP_ENV_NODE1){
		if(ledr <= strategy_env2_off[1].temp){
			rpm = strategy_env2_off[0].rpm;
			env2_node[1] = FALSE;
			env2_node[2] = FALSE;
 		}else if(ledr <= strategy_env2_on[1].temp){
			if(env2_node[1])
				rpm = strategy_env2_off[1].rpm;
			else
				rpm = strategy_env2_on[0].rpm;
		
			env2_node[2] = FALSE;
 		}else if(ledr <= strategy_env2_off[2].temp){
			rpm = strategy_env2_on[1].rpm;
			env2_node[1] = TRUE;
			env2_node[2] = FALSE;
 		}else if(ledr <= strategy_env2_on[2].temp){
			if(env2_node[2])
				rpm = strategy_env2_off[2].rpm;
			else
				rpm = strategy_env2_on[1].rpm;
			
			env2_node[1] = TRUE;
 		}else{
			//出现温度预警事件
			if(fun_tick_finish()){
				warning |= TEMP_LIMIT;
				fun_tick_start(DELAY_FUN);
			}
			
			rpm = strategy_env2_on[2].rpm;
			env2_node[1] = TRUE;
			env2_node[2] = TRUE;
 		}
		
	}else{
		//出现温度预警事件
		if(fun_tick_finish()){
			warning |= TEMP_LIMIT;
			fun_tick_start(DELAY_FUN);
		}
	}

//调节风扇转速,如果出现温度告警，转动1min后，切换到最低转速
	if(!fun_tick_finish())
		regulate_fun(FUN_MAX_SPEED);
	else
		regulate_fun(rpm);
}

