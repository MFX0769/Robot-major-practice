/****************************************************************************
	*	@笔者	：	星甲 
	*	@日期	：	2024年5月
	*	@所属	：	星呗科技
	*	@论坛	：	https://xbrobot.taobao.com
 ****************************************************************************/

#ifndef __MAIN_H__
#define __MAIN_H__

#include "z_setup.h"
#include "z_loop.h"

/*******全局变量宏定义*******/
#define UART_RECEIVE_BUF_SIZE  100

/*******全局变量外部声明*******/
extern u8 i;
extern u8 uart_receive_buf[UART_RECEIVE_BUF_SIZE], uart_receive_buf_index, uart_get_ok;

#endif 
