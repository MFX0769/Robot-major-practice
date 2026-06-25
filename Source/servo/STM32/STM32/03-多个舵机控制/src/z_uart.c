/****************************************************************************
	*	@笔者	：	星甲  
	*	@日期	：	2024年5月
	*	@所属	：	星呗科技
	*	@论坛 ：	https://xbrobot.taobao.com
	*	@功能	：	存放串口相关的函数
	*	@函数列表：
	*	1.	void uart1_init(u32 baud) -- 初始化串口1
	*	2.	void uart3_init(u32 baud) -- 初始化串口3
	*	3.	void uart1_send_byte(u8 dat) -- 串口1发送字节
	*	4.	void uart3_send_byte(u8 dat) -- 串口3发送字节
	*	5.	void uart1_send_str(char *s) -- 串口1发送字符串
	*	6.	void uart3_send_str(char *s) -- 串口3发送字符串
 ****************************************************************************/

#include "z_uart.h"

/***********************************************
	函数名称:	uart1_init() 
	功能介绍:	初始化串口1
	函数参数:	baud 波特率
	返回值:		无
 ***********************************************/
void uart1_init(u32 baud) {
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
	USART_DeInit(USART1);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
	USART_ClockInit(USART1, &USART_ClockInitStructure); 

	USART_InitStructure.USART_BaudRate = baud;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure ); 

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	USART_Cmd(USART1, ENABLE);
	uart1_open();
}

/***********************************************
	函数名称:	uart3_init() 
	功能介绍:	初始化串口3
	函数参数:	baud 波特率
	返回值:		无
 ***********************************************/
void uart3_init(u32 baud) {
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
	USART_ClockInit(USART3, &USART_ClockInitStructure);

	USART_InitStructure.USART_BaudRate = baud;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

	USART_Cmd(USART3, ENABLE);
	uart3_open();
}

/***********************************************
	函数名称:	uart1_send_byte() 
	功能介绍:	串口1发送字节
	函数参数:	dat 发送的字节
	返回值:		无
 ***********************************************/
void uart1_send_byte(u8 dat) {
	USART_SendData(USART1, dat);
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	return;
}

/***********************************************
	函数名称:	uart3_send_byte() 
	功能介绍:	串口3发送字节
	函数参数:	dat 发送的字节
	返回值:		无
 ***********************************************/
void uart3_send_byte(u8 dat) {
	USART_SendData(USART3, dat);
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET); 
	return;
}

/***********************************************
	函数名称:	uart1_send_str() 
	功能介绍:	串口1发送字符串
	函数参数:	*s 发送的字符串
	返回值:		无
 ***********************************************/
void uart1_send_str(u8 *s) {
	uart1_close();
	while (*s) {
		uart1_send_byte(*s++);
	}
	uart1_open();
}

/***********************************************
	函数名称:	uart3_send_str() 
	功能介绍:	串口3发送字符串
	函数参数:	*s 发送的字符串
	返回值:		无
 ***********************************************/
void uart3_send_str(u8 *s) {
	uart3_close();
	while (*s) {
		uart3_send_byte(*s++);
	}
	uart3_open();
}

/***********************************************
	函数名称:		void USART1_IRQHandler(void)
	功能介绍:		串口1中断函数
	函数参数:		无
	返回值:		无
 ***********************************************/
void USART1_IRQHandler(void) {
	if(USART_GetFlagStatus(USART1,USART_IT_RXNE)==SET) {
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
	return;
}

/***********************************************
	函数名称:	void USART3_IRQHandler(void) 
	功能介绍:	串口3中断函数
	函数参数:	无
	返回值:		无
 ***********************************************/
void USART3_IRQHandler(void) {
	if(USART_GetFlagStatus(USART3,USART_IT_RXNE)==SET) {
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
	}
	return;
}
