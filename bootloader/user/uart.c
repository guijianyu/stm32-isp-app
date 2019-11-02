#include "uart.h"
#include <string.h>
#include <stdio.h>

//usart1用作调试接口
static void USART1_Config(void){
	USART_InitTypeDef USART_InitStructure;
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
	//NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	//NVIC_InitStructure.NVIC_IRQChannelPriority = 3;
	//NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//NVIC_Init(&NVIC_InitStructure);

	/* Enable USART */
	USART_Cmd(USART1, ENABLE);

	//USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接收中断
}

//usart2用作与msd6a648通讯接口
static void USART2_Config(void){
	USART_InitTypeDef USART_InitStructure;
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
	//NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	//NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
	//NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//NVIC_Init(&NVIC_InitStructure);

	/* Enable USART */
	USART_Cmd(USART2, ENABLE);

	//USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启串口接收中断
	
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

static void _USART_Transfer(USART_TypeDef* USARTx, uint8_t* data, uint8_t len)
{
	uint8_t i;
	for(i = 0; i < len; i++){
		USART_SendData(USARTx, data[i]);
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
	}
}

//static void USART2_Transfer(uint8_t* data, uint8_t len)
//{
//	uint8_t i;
//	for(i = 0; i < len; i++){
//		USART_SendData(USART2, data[i]);
//		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
//	}
//}

void USART_Transfer(uint8_t* data, uint8_t len)
{
	_USART_Transfer(USART1, data, len);
	_USART_Transfer(USART2, data, len);
}

