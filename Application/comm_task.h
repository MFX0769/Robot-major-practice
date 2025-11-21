#ifndef __COMM_TASK_H__
#define __COMM_TASK_H__

#include "stm32h7xx.h"

typedef struct
{
//////    /* 4 chassis motor current */
//////    int16_t chassis_cur[4];
    /* yaw/pitch motor current */
    int16_t gimbal_cur[2];
    /* stir current */
    int16_t trigger_cur;
	//发射器测试代码，暂不清楚id与can口
	int16_t fric_cur[2];

} motor_current_t;

extern motor_current_t motor_cur;

void can_msg_send_task(void const *argu);
void usart_msg_send_task(void const *argu);
void judge_send_task(void const *argu);

extern motor_current_t motor_cur;
extern uint32_t can_send_cnt;

#endif
