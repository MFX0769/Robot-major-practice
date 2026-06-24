/**
  * @file    chassis_task.h
  * @brief   底盘控制任务头文件
  */

#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#include <stdint.h>

/* 底盘电机结构体 */
typedef struct {
    int16_t motor[4];  /* 4个电机电流值: [0]=FL, [1]=FR, [2]=BL, [3]=BR */
} chassis_motor_t;

/* 底盘专用模式，跟全局ctrl_mode是两个独立概念，互不影响。由SBUS sw2(ch7)单独驱动 */
typedef enum {
    CHASSIS_STOP,      /* 静止，给机械臂调试用 */
    CHASSIS_REMOTER,   /* 遥控器直驱 */
    CHASSIS_LINE_TRACK /* 巡线 */
} chassis_mode_e;

/* 底盘轴距系数 */
#define MECANUM_L  1.0f

/* 函数声明 */
void chassis_task(void const *argu);

#endif
