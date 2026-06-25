/**
  * @file    arm_task.h
  * @brief   机械臂主任务
  * @note    状态机 + 流程编排 + 视觉输入
  *          yaw用GM6020 ECD闭环PID，肩/肘/夹爪用PWM舵机
  */

#ifndef __ARM_TASK_H__
#define __ARM_TASK_H__

#include <stdint.h>
#include "arm_ik.h"
#include "pid.h"

/* Includes ------------------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* 机械臂调试打印开关 */
#define ARM_DBG_ENABLE     1   /* 1=使能调试打印, 0=禁用 */

/* IK自检开关 */
#define ARM_IK_SELFTEST_EN 1   /* 1=启动时跑IK自检, 0=跳过 */

/* 调试打印宏 (通过USART6输出, 波特率921600) */
#if ARM_DBG_ENABLE
    extern int dbg_print(const char *fmt, ...);
    #define ARM_DBG(fmt, ...)   dbg_print(fmt, ##__VA_ARGS__)
#else
    #define ARM_DBG(fmt, ...)
#endif

/* 任务周期 (ms) */
#define ARM_TASK_PERIOD  3

/* Yaw电流限幅 (6020电压模式: ±25000) */
#define ARM_YAW_CURRENT_LIMIT  30000

/* Yaw硬限位ecd (2026-06-25实测，原始ecd绝对值，跟offset_ecd/session无关，参考gimbal_task.h的YAW_ECD_LEFT/RIGHT模式) */
#define ARM_YAW_ECD_LEFT       1018    /* 左极限(逆时针最大) */
#define ARM_YAW_ECD_RIGHT      3988    /* 右极限(顺时针最大) */
#define ARM_YAW_LIMIT_KP       10.0f   /* 限位回拉系数（ecd超1tick→output=10） */

/* ECD转角度: 8192 = 360度 */
#define ECD_TO_DEG  (360.0f / 8192.0f)
#define DEG_TO_ECD  (8192.0f / 360.0f)

/* 平滑步进常量 */
#define ARM_SMOOTH_STEP_DEBUG   0.01f   /* 调试模式: 0.01度/周期 (每秒3.3度) */
#define ARM_SMOOTH_STEP_NORMAL  1.0f    /* 正常模式: 1.0度/周期 (每秒333度) */

/* Exported types ------------------------------------------------------------*/

/* 机械臂状态 (switch case, 7种) */
typedef enum {
    ARM_IDLE = 0,       /* 空闲，等待指令 */
    ARM_READY,          /* 就绪，可接受目标 */
    ARM_MOVING,         /* 运动中，IK+PID+平滑 */
    ARM_GRIP,           /* 夹取 */
    ARM_RELEASE,        /* 释放 */
    ARM_HOME,           /* 回零 */
    ARM_ERROR,          /* 错误，舵机断电 */
} arm_state_e;

/* 平滑模式 */
typedef enum {
    ARM_SMOOTH_DEBUG  = 0,   /* 调试模式: 超慢, 可随时打保护 */
    ARM_SMOOTH_NORMAL = 1,   /* 正常模式: 快速响应 */
} arm_smooth_mode_e;

/* yaw ECD闭环PID (参考gimbal_task.c的ecd写法) */
typedef struct {
    pid_t   angle_pid;      /* 位置环: fdb=ecd转角度, ref=目标角度 */
    pid_t   spd_pid;        /* 速度环: fdb=speed_rpm, ref=位置环输出 */
    float   angle_ref;      /* 目标角度 (度) */
    float   angle_fdb;      /* 反馈角度 (度, 从ecd换算) */
    float   spd_ref;        /* 速度目标 (来自位置环) */
    float   spd_fdb;        /* 速度反馈 (rpm) */
    float   output;         /* 最终输出电流 */
    int16_t current_limit;  /* 电流限幅 */
} arm_yaw_t;

/* 机械臂控制结构体 */
typedef struct {
    arm_state_e state;
    arm_joint_t target;      /* 目标角度 */
    arm_joint_t current;     /* 当前角度 */
    arm_pos_t target_pos;    /* 目标位置 (来自视觉) */
    uint8_t pos_valid;       /* 位置有效标志 */
    uint8_t vision_valid;    /* 视觉数据有效标志 */
    float smooth_step;       /* 平滑步进 (度/周期) */
    arm_yaw_t yaw;           /* yaw PID控制 */
} arm_ctrl_t;

/* Exported variables --------------------------------------------------------*/

extern arm_ctrl_t arm_ctrl;

/* Exported function declarations --------------------------------------------*/

/* 主任务 */
void arm_task_init(void);
void arm_task(void);

/* 目标设置 */
void arm_set_target_pos(float x, float y, float z);
void arm_set_target_yaw(float yaw_deg);
void arm_set_smooth_mode(arm_smooth_mode_e mode);

/* 状态查询/设置 */
arm_state_e arm_get_state(void);
void arm_set_state(arm_state_e state);

/* 视觉输入 */
void arm_vision_update(void);

/* Yaw PID */
void arm_yaw_pid_init(void);
void arm_yaw_pid_calc(void);

/* 调试 */
void arm_debug_info(void);
void arm_ik_self_test(void);

#endif /* __ARM_TASK_H__ */
