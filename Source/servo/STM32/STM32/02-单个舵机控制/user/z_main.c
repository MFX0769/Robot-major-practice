/****************************************************************************
	*	@笔者	：	星甲
	*	@日期	：	2024年5月
	*	@所属	：	星呗科技
	*	@店铺 ：	https://xbrobot.taobao.com
	*	@功能	：	KSP控制板（32）总线舵机的基本控制
	*	@函数列表:
	*	1.	void main(void) -- 主函数
 ****************************************************************************/

#include "z_main.h"

/*******全局变量定义*******/
u8 i=0;
u8 uart_receive_buf[UART_RECEIVE_BUF_SIZE]={0}, uart_receive_buf_index, uart_get_ok;

int main(void) {																																																																																																																															//ZL
	setup_systick();		//初始化滴答时钟，1ms增加一次systick_ms的值
	setup_led();			//初始化LED信号灯
	setup_beep();			//初始化蜂鸣器BEEP
	setup_uart1();			//初始化串口1
	setup_uart3();			//初始化串口2
	setup_finish();			//初始化完成

/**********************************************
	
 **********************************************/
	uart3_send_str((u8 *)"#255PID000!");
	delay_ms(500);
	uart3_send_str((u8 *)"#000PID!");
	delay_ms(500);
	uart3_send_str((u8 *)"#000P2500T1000!");
	delay_ms(500);
	uart3_send_str((u8 *)"#000P1500T1000!");
	
	while (1) {
	}
}
