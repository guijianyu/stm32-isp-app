#include "uart.h"
#include <string.h>
#include <stdio.h>
#include "types.h"

uint8_t TxBuffer[BUFFERSIZE];
uint8_t RxBuffer[BUFFERSIZE];
uint8_t NbrOfDataToRead = BUFFERSIZE;
__IO uint8_t TxCount = 0;
__IO uint16_t RxCount = 0;
__IO uint32_t TimeOut = 0x00; 
__IO uint8_t mode = 0;
struct ringBuffer ringbuf;

//usart1用作调试接口
static void USART1_Config(void){
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIO clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	/* Enable USART clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* Connect PXx to USARTx_Tx */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);

	/* Connect PXx to USARTx_Rx */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);

	/* Configure USART Tx and Rx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* USARTx configuration ----------------------------------------------------*/
	/* USARTx configured as follow:
	- BaudRate = 115200 baud  
	- Word Length = 8 Bits
	- One Stop Bit
	- No parity
	- Hardware flow control disabled (RTS and CTS signals)
	- Receive and transmit enabled
	*/
	USART_InitStructure.USART_BaudRate = UART_BAUD_RATE;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	/* NVIC configuration */
	/* Enable the USARTx Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable USART */
	USART_Cmd(USART1, ENABLE);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接收中断
}

//usart2用作与msd6a648通讯接口
static void USART2_Config(void){
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIO clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	/* Enable USART clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* Connect PXx to USARTx_Tx */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);

	/* Connect PXx to USARTx_Rx */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);

	/* Configure USART Tx and Rx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* USARTx configuration ----------------------------------------------------*/
	/* USARTx configured as follow:
	- BaudRate = 115200 baud  
	- Word Length = 8 Bits
	- One Stop Bit
	- No parity
	- Hardware flow control disabled (RTS and CTS signals)
	- Receive and transmit enabled
	*/
	USART_InitStructure.USART_BaudRate = UART_BAUD_RATE;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);

	/* NVIC configuration */
	/* Enable the USARTx Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable USART */
	USART_Cmd(USART2, ENABLE);

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启串口接收中断
	
	/* Enable the EVAL_COM1 Transmoit interrupt: this interrupt is generated when the 
	EVAL_COM1 transmit data register is empty */
	//USART_ITConfig(USART2, USART_IT_TXE, ENABLE);

	/* Wait until EVAL_COM1 send the TxBuffer */
	//while(TxCount < NbrOfDataToTransfer){}

}

void USART_Config(void)
{ 
	USART1_Config();
	USART2_Config();
}

void SysTickConfig(void)
{
	/* Setup SysTick Timer for 1ms interrupts  */
	if (SysTick_Config(SystemCoreClock / SYSTICK_PERIOD))
		/* Capture error */
		while (1);

	/* Configure the SysTick handler priority */
	NVIC_SetPriority(SysTick_IRQn, 0x0);
}

static inline void _USART_Transfer(USART_TypeDef* USARTx, uint8_t* data, uint8_t len)
{
	uint8_t i;
	for(i = 0; i < len; i++){
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);				
		USART_SendData(USARTx, data[i]);		
	}
}

void USART_Transfer(uint8_t* data, uint8_t len)
{
	_USART_Transfer(USART1, data, len);
	_USART_Transfer(USART2, data, len);
}

//1s接收到8个字节，存入缓冲区，否则丢弃
void USART_Receive(USART_TypeDef* USARTx, uint16_t data)
{
	if(mode == FRAME_RECEIVE_PREPARED){
		TimeOut = USER_TIMEOUT;
		mode = FRAME_RECEIVE_PROCESSING;
	}
	
	/* Read one byte from the receive data register */
	RxBuffer[RxCount++] = data;
	if(USARTx == USART2)
		_USART_Transfer(USART1, &RxBuffer[RxCount - 1], 1);
	
	if(RxCount == NbrOfDataToRead){
		RxCount = 0;
		mode = FRAME_RECEIVE_COMPELTED;
		memcpy(&ringbuf.buf[ringbuf.tail], RxBuffer, NbrOfDataToRead);
		if(++ringbuf.tail >= RING_SIZE)			//尾节点偏移
			ringbuf.tail = 0;									//大于数组最大长度 归零 形成环形队列
		if(ringbuf.tail == ringbuf.head)	//如果尾部节点追到头部节点，则修改头节点偏移位置丢弃早期数据
			if(++ringbuf.head >= RING_SIZE)
				ringbuf.head = 0;

		//memcpy(TxBuffer, RxBuffer, NbrOfDataToRead);
		memset(RxBuffer, 0, BUFFERSIZE);
		//USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
	}

}

struct msg* buffer_stat(){
	struct msg* m = NULL;
	
	switch(mode){
	case FRAME_RECEIVE_PREPARED:
		
		break;
	
	case FRAME_RECEIVE_PROCESSING:
		//超时，丢弃RxBuffer中已接收数据
		if(TimeOut == 0){
			memset(RxBuffer, 0, BUFFERSIZE);
			RxCount = 0;
			mode = FRAME_RECEIVE_PREPARED;
		}
		break;
		
	case FRAME_RECEIVE_COMPELTED:
	//	//未超时，存入环形缓冲区
	//	//process((struct msg*)TxBuffer);
		mode = FRAME_RECEIVE_PREPARED;
	//	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
		break;
	}

	
	if(ringbuf.head !=ringbuf.tail)
	{
		//USART_Transfer(&ringbuf.buf[ringbuf.head], BUFFERSIZE);
		m = &ringbuf.buf[ringbuf.head];
		if(++ringbuf.head >= RING_SIZE)
			ringbuf.head = 0;
	}

	return m;
}

/**********禁用ARM的半主机工作模式**********/
#pragma import(__use_no_semihosting)                             
struct __FILE { 
    int handle; 
}; 

FILE __stdout;
void _sys_exit(int x) 
{ 
	
}

int fputc(int ch, FILE *f)
{
	uint8_t data = (unsigned char)ch;
	_USART_Transfer(USART1, &data, 1);
	//while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);		//等待上次发送结束	
	//USART_SendData(USART1, (unsigned char)ch);				//发送数据到串口	
	return ch;
}

uint8_t sum(uint8_t* data, uint8_t len)
{
	uint16_t sum = 0, i;
	for(i = 0; i< len; i++){
		sum += data[i];
	}
	return (uint8_t)(sum & 0xFF);
}

