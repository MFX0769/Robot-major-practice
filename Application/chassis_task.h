/**
  * @file    chassis_task.h
  * @brief   底盘控制任务头文件
  */

#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#include <stdint.h>
#include "pid.h"
#include "prot_rm_motor.h"
#include "prot_8irtraking.h"

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

/* ---- 控制台调试钩子（单电机开环点动 / PID 在线调参） ---- */
#define CHASSIS_JOG_MAX_CUR  3000    /* 2006 电流给定上限，C610 满量程 10000 */
#define CHASSIS_JOG_MS       1500U   /* 一次点动最长持续时间，到时自动归零 */

void    chassis_dbg_jog_apply(dji_motor_t *m);            /* 由控制环在临界区内调用 */
int     chassis_dbg_jog_start(int32_t idx, int32_t cur);   /* idx 0..3 */
void    chassis_dbg_jog_stop(void);
uint8_t chassis_dbg_jog_active(void);
pid_t  *chassis_dbg_pid_get(uint8_t i);                   /* i 0..3，越界返回 0 */
pid_t  *chassis_dbg_line_pid_get(void);

/* ---- 摇杆方向微调（标定块 reserved[0..2] 掉电保存） ---- */
extern int8_t g_chassis_map_sign[3];                      /* 0=vx 1=vy 2=wz，值 ±1 */
void    chassis_map_sign_set(uint8_t idx, int8_t sign);   /* 由 arm_task 的 cal_apply 调 */

/* ---- 前向测距避障（标定块 reserved[3..5]：kodo / 横移 / 前进，掉电保存） ---- */
void    chassis_av_init(void);                            /* 底盘任务启动时调 */
void    chassis_av_apply(uint16_t kodo_x1000, uint16_t shift_mm, uint16_t fwd_mm, int8_t side);   /* cal_apply 调 */
void    chassis_odo_step(void);                           /* 每圈调：轮速积分 */
void    chassis_av_tick(float *vx, float *vy, float *wz, const prot_8irtraking_data_t *line);   /* 巡线分支里调 */
void    chassis_av_cancel(void);                          /* 离开巡线模式时调 */
void    chassis_av_cmd(const char *sub, const char *arg); /* 控制台 avoid ... */

#endif
