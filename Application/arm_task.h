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

#define ARM_YAW_OUT_LIMIT      25000   /* 6020 0x1FF 电压帧硬上限 */
#define ARM_YAW_OUT_LIMIT_DFLT 8000    /* 未整定前的默认限幅，yaw limit 可放宽到硬上限 */
#define ARM_TEACH_MAX_POINTS   512U
#define ARM_TEACH_SAMPLE_MS    50U     /* 20Hz，也是回放的时间单位 */
#define ARM_YAW_CMD_HOLD_MS    10000U  /* 控制台 yaw 命令保持时间，超时归零 */
#define ARM_YAW_FB_TIMEOUT_MS  100U    /* yaw 反馈超时 */
#define ARM_TEACH_DELETE_CONFIRM_MS 12000U

/* arm_ik.c 仍在用这个打印宏 */
#ifndef ARM_DBG
extern int dbg_print(const char *fmt, ...);
#define ARM_DBG(fmt, ...)   dbg_print(fmt, ##__VA_ARGS__)
#endif

/* yaw 故障码 */
#define ARM_YAW_FAULT_RUNAWAY  1U      /* 远离目标运动：方向接反 */
#define ARM_YAW_FAULT_FB_LOST  2U

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
    pid_t   spd_pid;        /* 速度环: rpm -> PID 输出 */
    uint8_t enable;         /* 1=闭环输出, 0=零输出 */
    uint8_t online;         /* 反馈新鲜 */
    uint8_t dir_valid;      /* 1=方向已标定（yaw probe / yaw dir） */
    int8_t  dir;            /* +1: 正电压使 ECD 增大; -1: 相反 */
    uint8_t fault;          /* ARM_YAW_FAULT_*，非 0 时锁零输出 */
    int32_t target_ecd;     /* 已夹到限位 */
    int32_t fdb_ecd;
    int32_t err_ecd;
    float   angle_ref;      /* deg */
    float   angle_fdb;
    float   spd_ref;
    float   spd_fdb;
    float   out_pid;        /* 乘 dir 之前的 PID 输出 */
    float   out;            /* 写入 0x1FF 的电压值 */
    int16_t out_limit;      /* 运行时限幅，<= ARM_YAW_OUT_LIMIT */
} arm_yaw_ctrl_t;

/* 轨迹点：10 字节，无填充 */
typedef struct {
    int16_t  yaw_ecd;
    uint16_t shoulder;
    uint16_t elbow;
    uint16_t gripper;
    uint8_t  dt50;          /* 距上一点的采样 tick 数（50ms/tick），首点为 0 */
    uint8_t  reserved;
} arm_teach_point_t;

typedef struct {
    arm_teach_state_e state;
    uint16_t count;
    uint16_t play_index;    /* 回放中当前段的起点下标 */
    uint8_t  origin_valid;
    int32_t  origin_ecd;
    uint8_t  saved_valid;   /* RAM 里的轨迹与 Flash 一致，可回放 */
    uint8_t  delete_confirm;
    uint32_t delete_deadline;
    uint32_t last_sample_tick;
    uint32_t state_tick;
    int32_t  play_max_err;  /* 回放期间 |target-fdb| 最大值, ECD */
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

/* 模式门控：只有 REMOTER(含控制台 mode manual) 和 ARM_TEACH 允许机械臂输出 */
uint8_t    arm_motion_allowed(void);
const char *arm_mode_name(void);
int  arm_mode_manual(void);        /* 控制台 mode manual */

/* yaw 双环。返回 0 成功，<0 被拒绝（原因已打印） */
int  arm_yaw_set_enable(uint8_t en);
int  arm_yaw_set_target_ecd(int32_t ecd);
int  arm_yaw_probe(void);          /* 开环小电压判方向，返回 +1/-1，0=没动，<0 拒绝 */
int  arm_yaw_set_dir(int32_t dir);
int  arm_yaw_set_limit(int32_t limit);
int  arm_yaw_set_speed_limit(int32_t rpm);
int  arm_yaw_set_gain(float pos_kp, float pos_kd, float spd_kp, float spd_kd);
void arm_yaw_fault_clear(void);
void arm_yaw_pid_calc(void);

/* 点动 / 停止 */
int  arm_joint_jog(uint8_t joint, const char *value);   /* 绝对脉宽，或 +50 / -50 相对步进 */
int  arm_joint_set_limit(uint8_t joint, int32_t lo, int32_t hi);
void arm_joint_status(void);
void arm_stop_all(void);

/* 示教 */
void arm_teach_cmd(const char *sub, const char *arg);
void arm_flash_test(void);         /* 不接 24V 也能做的片内 Flash 验证 */
const char *arm_teach_state_name(void);
uint8_t arm_teach_is_recording(void);

#endif /* __ARM_TASK_H__ */
