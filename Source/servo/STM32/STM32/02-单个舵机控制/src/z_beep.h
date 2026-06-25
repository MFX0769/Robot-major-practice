/****************************************************************************
	*	@笔者	：	星甲
	*	@日期	：	2024年5月
	*	@所属	：	星呗科技
	*	@论坛	：	https://xbrobot.taobao.com
 ****************************************************************************/

#ifndef __BEEP_H__
#define __BEEP_H__

#include "stm32f10x.h"

/*******IO口映射表*******/
#define BEEP_Pin			GPIO_Pin_5
#define BEEP_GPIO_Port		GPIOB

/*******BEEP快捷指令表*******/
#define BEEP				GPIO_ReadOutputDataBit(BEEP_GPIO_Port,BEEP_Pin)	//读取蜂鸣器BEEP状态
#define BEEP_H()			GPIO_ResetBits(BEEP_GPIO_Port,BEEP_Pin)			//蜂鸣器BEEP打开
#define BEEP_L()			GPIO_SetBits(BEEP_GPIO_Port,BEEP_Pin)			//蜂鸣器BEEP关闭

/*******BEEP相关函数声明*******/
void beep_init(void);		//蜂鸣器BEEP初始化
void BEEP_F(void);			//翻转蜂鸣器BEEP

#endif
