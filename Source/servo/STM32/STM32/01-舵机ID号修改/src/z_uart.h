/****************************************************************************
	*	@笔者	：	星甲
	*	@日期	：	2024年5月
	*	@所属	：	星呗科技
	*	@论坛 ：	https://xbrobot.taobao.com
 ****************************************************************************/

#ifndef __UART_H__
#define __UART_H__

#include <string.h>
#include "stm32f10x.h"
#include "z_main.h"

/*******串口快捷指令表*******/
#define uart1_open()  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE)
#define uart3_open()  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE)
#define uart1_close() USART_ITConfig(USART1, USART_IT_RXNE, DISABLE)
#define uart3_close() USART_ITConfig(USART3, USART_IT_RXNE, DISABLE)

/*******串口相关函数声明*******/
void uart1_init(u32 baud);
void uart1_send_byte(u8 dat);
void uart1_send_str(u8 *s);

void uart3_init(u32 baud);
void uart3_send_byte(u8 dat);
void uart3_send_str(u8 *s);

#endif
