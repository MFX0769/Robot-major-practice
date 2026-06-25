/****************************************************************************
	*	@笔者	：	星甲
	*	@日期	：	2024年5月
	*	@所属	：	星呗科技
	*	@论坛	：	https://xbrobot.taobao.com
	*	@功能	：	存放LED信号灯相关的函数
	*	@函数列表：
	*	1.	void led_init(void) -- 初始化LED信号灯
	*	2.	void LED_F(void) -- 翻转LED信号灯
 ****************************************************************************/

#include "z_led.h"

/***********************************************
	函数名称:	led_init() 
	功能介绍:	初始化LED信号灯
	函数参数:	无
	返回值:		无
 ***********************************************/
void led_init(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = LED_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LED_GPIO_Port, &GPIO_InitStructure);
}

/***********************************************
	函数名称:	LED_F() 
	功能介绍:	翻转LED信号灯
	函数参数:	无
	返回值:		无
 ***********************************************/
void LED_F(void) {
	if(LED==0)	
		LED_L();
	else 
		LED_H();
}
