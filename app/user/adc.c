
#include "adc.h"
#include <stdio.h>
#include <math.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADC1_DR_Address                0x40012440
__IO uint16_t RegularConvData_Tab[5];   //ADC1 channelx 转换的温度传感器电压值通过DMA方式放入内存中

/*
*********************************************************************************************************
*	函 数 名: ADC_Config_Init
*	功能说明: ADC模块初始化配置
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void ADC_Config_Init(void)
{
	ADC_InitTypeDef     ADC_InitStruct;
	DMA_InitTypeDef     DMA_InitStruct;
	GPIO_InitTypeDef    GPIO_InitStruct;

	/* ADC1 DeInit */  
	ADC_DeInit(ADC1);

	/* Enable  GPIO clock */
	RCC_AHBPeriphClockCmd(ADC_ALL_GPIO_CLK, ENABLE);

	/* ADC1 Periph clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	/* DMA1 clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE);

	/* Configure gpio pin  as analog input */
	GPIO_InitStruct.GPIO_Pin = TMEP_SENSOR_0_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(TEMP_SENSOR_0_PORT, &GPIO_InitStruct);		// GPIO引脚，输入时不用设置速率

	GPIO_InitStruct.GPIO_Pin = TMEP_SENSOR_1_PIN;
	GPIO_Init(TEMP_SENSOR_1_PORT, &GPIO_InitStruct);		// GPIO引脚，输入时不用设置速率

	GPIO_InitStruct.GPIO_Pin = LIGHT_SENSOR_0_PIN;
	GPIO_Init(LIGHT_SENSOR_0_PORT, &GPIO_InitStruct);		// GPIO引脚，输入时不用设置速率

	GPIO_InitStruct.GPIO_Pin = LIGHT_SENSOR_1_PIN;
	GPIO_Init(LIGHT_SENSOR_1_PORT, &GPIO_InitStruct);		// GPIO引脚，输入时不用设置速率	
	
	GPIO_InitStruct.GPIO_Pin = DLP_IADJ_ADC_PIN;
	GPIO_Init(DLP_IADJ_ADC_PORT, &GPIO_InitStruct);			// GPIO引脚，输入时不用设置速率		

	/* Initialize ADC structure */
	ADC_StructInit(&ADC_InitStruct);
	
	/* Configure the ADC1 in continous mode withe a resolutuion equal to 12 bits  */
	ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStruct.ADC_ContinuousConvMode = ENABLE; 
	ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_ScanDirection = ADC_ScanDirection_Upward;	/*from CHSEL0 to CHSEL18*/
	ADC_Init(ADC1, &ADC_InitStruct); 

	/* Convert the ADC1 Channel0 1 4 5 6--  with 55.5 Cycles as sampling time */ 
	ADC_ChannelConfig(ADC1, TMEP_SENSOR_0_ADC_CHANNEL, ADC_SampleTime_55_5Cycles); 
	ADC_ChannelConfig(ADC1, TMEP_SENSOR_1_ADC_CHANNEL, ADC_SampleTime_55_5Cycles); 
	ADC_ChannelConfig(ADC1, LIGHT_SENSOR_0_ADC_CHANNEL, ADC_SampleTime_55_5Cycles); 
	ADC_ChannelConfig(ADC1, LIGHT_SENSOR_1_ADC_CHANNEL, ADC_SampleTime_55_5Cycles); 	
	ADC_ChannelConfig(ADC1, DLP_IADJ_ADC_CHANNEL, ADC_SampleTime_55_5Cycles); 	

	/* ADC Calibration */
	ADC_GetCalibrationFactor(ADC1);

	/* ADC DMA request in circular mode */
	ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);

	/* Enable ADC_DMA */
	ADC_DMACmd(ADC1, ENABLE); 
	
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);     

	/* Wait the ADRDY falg */
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY)); 

	/* ADC1 regular Software Start Conv */ 
	ADC_StartOfConversion(ADC1);

	/* DMA1 Channel1 Config */
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;
	DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)RegularConvData_Tab;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStruct.DMA_BufferSize = 5;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStruct.DMA_Priority = DMA_Priority_High;
	DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStruct);

	/* DMA1 Channel1 enable */
	DMA_Cmd(DMA1_Channel1, ENABLE);	
}

/*
*********************************************************************************************************
*	函 数 名: ADC_Config_Init
*	功能说明: ADC模块初始化配置
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
double get_temperauter_value(int number)
{
	double ADC_Voltage_mv=0; 	  	//adc channel xxx 电压，单位MV
	double resistance_value = 0;  	//温度传感器的电阻值
	double temperature_value = 0;	//记录温度值
	
	if(number == TEMP_SENSOR_0)
	{
		ADC_Voltage_mv = (double)RegularConvData_Tab[0]*3300/0xFFF;   //计算ADC引脚测量的电压值，单位mv
		resistance_value = (10000*(double)ADC_Voltage_mv)/(3300-(double)ADC_Voltage_mv); //通过电路图得到计算公式，计算热敏电阻当前电阻值
		/*
			double log (double); 以e为底的对数
			double log10 (double);以10为底的对数
		*/
		if(resistance_value < 51000)  //大于40度
		{
			//通过NXRT15WF104FA3A016芯片 数据表得到公式
			//x电阻值，单位千欧，y温度		 
			//40°c到70°c的拟合曲线 y = -24.83ln(x) + 137.11
			//11°c到39°c的拟合曲线 y = y = -21.13ln(x) + 122.51
			//-40°c到10°c的拟合曲线y = -16.45ln(x) + 96.577
			temperature_value = (-24.14)*log(resistance_value/1000) + 135.37;  
		}
		else if (resistance_value >=51000 && resistance_value <= 200000)	//11到39度
		{
			temperature_value = (-21.13)*log(resistance_value/1000) + 122.51;
		}
		else 
		{
			temperature_value = (-16.45)*log(resistance_value/1000) + 96.577;
		}
	}
	else if (number == TEMP_SENSOR_1)
	{
		ADC_Voltage_mv = (double)RegularConvData_Tab[1]*3300/0xFFF;	 //计算ADC引脚测量的电压值，单位mv
		resistance_value = (10000*(double)ADC_Voltage_mv)/(3300-(double)ADC_Voltage_mv); //通过电路图得到计算公式，计算热敏电阻当前电阻值
		
		if(resistance_value < 54000)//大于40度
		{
			temperature_value = (-27.63)*log(resistance_value/1000) + 149.23;  //MF51B12-104F3950L32 excle文件计算得到
																	  //x电阻值，单位千欧，y温度	 	
																	  //40°c到70°c的拟合曲线 y = -27.63ln(x) + 149.23
																	  //11°c到39°c的拟合曲线 y = -22.81ln(x) + 130.31
																	  //-40°c到10°c的拟合曲线y = -18.54ln(x) + 107.45	
		}
		else if (resistance_value >= 54000 && resistance_value <= 191000)
		{
			temperature_value = (-22.81)*log(resistance_value/1000) + 130.31;
		}
		else 
		{
			temperature_value = (-18.54)*log(resistance_value/1000) + 107.45;
		}
	}

	#ifdef DEBUG_TEST
	//temperature Sensor data test
	printf("Temperature Sensor %d temper=%d\r\n",number,RegularConvData_Tab[number]);
	printf("Temperature Sensor %d voltage=%f\r\n",number,ADC_Voltage_mv);
	printf("Temperature Sensor %d Resistance=%f\r\n",number,resistance_value);
	printf("Temperature Sensor %d temper_value=%f\r\n",number,temperature_value);
	#endif 
	
	return temperature_value;
}


double get_light_value(int number)
{
//	double ADC_Voltage_mv=0; 	  	//adc channel xxx 电压，单位MV
//	double resistance_value = 0;  	//光传感器的电阻值
	uint16_t light_intensity = 0;	//光强度

	if(number == LIGHT_SENSOR_0)
	{
//		ADC_Voltage_mv = (double)RegularConvData_Tab[2]*3300/0xFFF;   //计算ADC引脚测量的电压值，单位mv
//		resistance_value = (10000*(double)ADC_Voltage_mv)/(3300-(double)ADC_Voltage_mv); //通过电路图得到计算公式，计算热敏电阻当前电阻值	
		light_intensity = 0xFFF - RegularConvData_Tab[2];		 
	}
	else if (number == LIGHT_SENSOR_1)
	{
//		ADC_Voltage_mv = (double)RegularConvData_Tab[3]*3300/0xFFF;	 //计算ADC引脚测量的电压值，单位mv
//		resistance_value = (10000*(double)ADC_Voltage_mv)/(3300-(double)ADC_Voltage_mv); //通过电路图得到计算公式，计算热敏电阻当前电阻值
		light_intensity = 0xFFF - RegularConvData_Tab[3];	
	}

	#ifdef DEBUG_TEST	
	//light Sensor data test
//	printf("light Sensor %d temper=%d\r\n",number,RegularConvData_Tab[number+2]);
//	printf("light Sensor %d voltage=%f\r\n",number,ADC_Voltage_mv);
//	printf("light Sensor %d Resistance=%f\r\n",number,resistance_value);
	printf("light intensity %d intensity=%d\r\n",number,light_intensity);
	#endif 
	
	return light_intensity;
}

double get_dlp_IADJ_voltage(void)
{
	double ADC_Voltage_mv=0; 	  	//adc channel xxx 电压，单位MV

	ADC_Voltage_mv = (double)RegularConvData_Tab[4]*3300/0xFFF;	 //计算ADC引脚测量的电压值，单位mv
	
	#ifdef DEBUG_TEST
	//dlp IADJ data test
	printf("dlp IADJ temper=%d\r\n",RegularConvData_Tab[4]);
	printf("dlp IADJ voltage=%f\r\n",ADC_Voltage_mv);	
	#endif 
	
	return ADC_Voltage_mv;
}

