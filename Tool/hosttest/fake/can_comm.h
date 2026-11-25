#ifndef FAKE_CAN_COMM_H
#define FAKE_CAN_COMM_H
/* 真头文件带着 HAL 的 CAN 类型；底盘任务只用到这一个函数 */
void chassis_can_send(void);
#endif
