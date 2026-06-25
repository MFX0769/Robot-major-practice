/****************************************************************************
	*	@笔者	：	星甲
	*	@日期	：	2024年5月
	*	@所属	：	星呗科技
	*	@论坛	：	https://xbrobot.taobao.com
 ****************************************************************************/

#ifndef __LED_H__
#define __LED_H__

#include "stm32f10x.h"

/*******IO口映射表*******/
#define LED_Pin				GPIO_Pin_13  
#define LED_GPIO_Port 		GPIOB

/*******LED快捷指令表*******/
#define LED					GPIO_ReadOutputDataBit(LED_GPIO_Port,LED_Pin)	//读取LED信号灯状态
#define LED_H()				GPIO_ResetBits(LED_GPIO_Port,LED_Pin)			//LED信号灯点亮
#define LED_L()				GPIO_SetBits(LED_GPIO_Port,LED_Pin)				//LED信号灯熄灭

/*******LED相关函数声明*******/
void led_init(void);		//初始化LED信号灯
void LED_F(void);			//翻转LED信号灯

#endif
