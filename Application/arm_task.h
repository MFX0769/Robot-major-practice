/**
  * @file    arm_task.h
  * @brief   机械臂：yaw 双环 + 单按钮拖动示教 + 四关节点动
  */
#ifndef __ARM_TASK_H__
#define __ARM_TASK_H__

#include <stdint.h>
#include "pid.h"

#ifndef ARM_TASK_PERIOD
#define ARM_TASK_PERIOD        3       /* ms */
#endif

#define ARM_YAW_OUT_LIMIT      25000   /* 6020 0x1FF 电压帧满量程 */
#define ARM_TEACH_MAX_POINTS   512U
#define ARM_TEACH_SAMPLE_MS    50U     /* 20Hz */
#define ARM_JOG_TIMEOUT_MS     500U

/* 兼容 arm_ik.c 的调试宏 */
#ifndef ARM_DBG
extern int dbg_print(const char *fmt, ...);
#define ARM_DBG(fmt, ...)   dbg_print(fmt, ##__VA_ARGS__)
#endif

typedef enum {
    ARM_TEACH_IDLE = 0,
    ARM_TEACH_READY,
    ARM_TEACH_RECORD,
    ARM_TEACH_SAVED,
    ARM_TEACH_PLAYBACK,
    ARM_TEACH_ABORT
} arm_teach_state_e;

typedef struct {
    pid_t   angle_pid;      /* 位置环: ECD->deg, 输出速度给定 */
    pid_t   spd_pid;        /* 速度环: rpm -> 0x1FF 输出 */
    uint8_t enable;         /* 1=闭环输出, 0=零输出 */
    uint8_t online;         /* 反馈新鲜 */
    int32_t target_ecd;     /* 已夹到限位 */
    int32_t fdb_ecd;
    int32_t err_ecd;
    float   angle_ref;      /* deg, 便于 Keil 观察 */
    float   angle_fdb;
    float   spd_ref;
    float   spd_fdb;
    float   out;
    int16_t out_limit;
} arm_yaw_ctrl_t;

typedef struct {
    int16_t  yaw_ecd;
    uint16_t shoulder;
    uint16_t elbow;
    uint16_t gripper;
} arm_teach_point_t;

typedef struct {
    arm_teach_state_e state;
    uint16_t count;
    uint16_t play_index;
    uint8_t  origin_valid;
    int32_t  origin_ecd;
    uint8_t  saved_valid;
    uint8_t  delete_confirm;
    uint32_t delete_deadline;
    uint32_t last_sample_tick;
    uint32_t state_tick;
    uint8_t  abort_reason;
    int32_t  play_max_err;   /* 回放期间 |target-fdb| 最大值, ECD */
} arm_teach_t;

typedef struct {
    arm_yaw_ctrl_t    yaw;
    arm_teach_t       teach;
} arm_ctrl_t;

extern arm_ctrl_t arm_ctrl;
extern arm_teach_point_t arm_teach_points[ARM_TEACH_MAX_POINTS];
extern uint16_t arm_teach_count;
extern volatile uint8_t arm_busy;

/* 任务 */
void arm_task(void const *argu);
void arm_task_init(void);

/* yaw 双环 */
void arm_yaw_set_enable(uint8_t en);
int  arm_yaw_set_target_ecd(int32_t ecd);
void arm_yaw_stop(void);
void arm_yaw_pid_calc(void);

/* 点动 / 停止 */
int  arm_joint_jog(uint8_t joint, uint16_t pulse);
void arm_stop_all(void);

/* 示教 */
void arm_teach_cmd(const char *sub);
const char *arm_teach_state_name(void);
uint8_t arm_teach_is_recording(void);
uint8_t arm_flash_ready(void);

#endif /* __ARM_TASK_H__ */
