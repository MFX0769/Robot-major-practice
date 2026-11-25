/**
  * @file    arm_task.h
  * @brief   机械臂：yaw 双环 + 单按钮拖动示教 + 四关节点动 + 死程序采摘 + 视觉 XY 逆解
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

#define ARM_PICK_STEPS         5U      /* 死程序采摘：就绪/伸出/夹合/收回/松开 */
#define ARM_PICK_KEEP_YAW      ((int16_t)-32768)   /* 路点里表示“yaw 保持当前位置” */

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

/* 标定、视觉折算、关节限位、yaw 参数。64 字节，和轨迹一起存片内 Flash，掉电不丢 */
typedef struct {
    uint16_t l1_mm;         /* 大臂长度 */
    uint16_t l2_mm;         /* 小臂长度 */
    uint16_t sh0_pulse;     /* 肩：大臂水平时的舵机脉宽 */
    uint16_t sh90_pulse;    /* 肩：大臂竖直向上时的脉宽 */
    uint16_t el0_pulse;     /* 肘：小臂与大臂共线时的脉宽 */
    uint16_t el90_pulse;    /* 肘：小臂相对大臂抬起 90 度时的脉宽 */
    uint16_t vis_depth_mm;  /* 视觉不给深度：果子相对肩关节的前向距离取这个固定值 */
    int16_t  cam_dy_mm;     /* 相机光轴相对肩关节的高度，向上为正 */
    uint16_t vis_k_x100;    /* 该深度下每像素对应的毫米数 x100 */
    int16_t  yaw_k_x100;    /* 横向像素偏差折成 yaw 修正（ECD/像素 x100），0 = 不动 yaw */
    uint16_t pick_open;     /* 夹爪张开脉宽 */
    uint16_t pick_close;    /* 夹爪闭合脉宽 */
    uint16_t ik_valid;      /* 非 0 = 四个标定点都已学过 */
    uint16_t flags;         /* bit0: yaw 方向已标定  bit1: yaw dir = -1 */
    uint16_t lim_sh_min;    /* 肩/肘/夹爪的关节限位，joint limit 改的就是这里 */
    uint16_t lim_sh_max;
    uint16_t lim_el_min;
    uint16_t lim_el_max;
    uint16_t lim_gr_min;
    uint16_t lim_gr_max;
    uint16_t yaw_out_limit;         /* 0 = 用编译时的默认值 */
    uint16_t yaw_speed_limit;       /* 位置环输出上限 rpm，0 = 默认 */
    uint16_t yaw_kp_pos_x100;       /* 四个增益，放大 100 倍存；0 = 默认 */
    uint16_t yaw_kd_pos_x100;
    uint16_t yaw_kp_spd_x100;
    uint16_t yaw_kd_spd_x100;
    uint16_t reserved[6];
} arm_cal_t;

#define ARM_CAL_FLAG_YAW_DIR_VALID  0x0001U
#define ARM_CAL_FLAG_YAW_DIR_NEG    0x0002U

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
    arm_yaw_ctrl_t      yaw;
    arm_teach_t         teach;
    arm_cal_t           cal;
    arm_teach_point_t   pick_wp[ARM_PICK_STEPS];   /* 死程序采摘的 5 个路点 */
    uint16_t            pick_ms[ARM_PICK_STEPS];   /* 走到该路点用的时间，[0] 不用 */
    uint8_t             pick_auto;                 /* 1 = pick run 时用最新一帧视觉重算“伸出”姿态 */
    int16_t             vis_x, vis_y;              /* 最近一帧视觉像素偏差 */
    uint32_t            vis_tick;
    uint8_t             manual;                    /* 1 = 控制台 mode manual：只放开机械臂，底盘锁零 */
    uint8_t             chassis_run;               /* 1 = chassis line 运行中，机械臂命令拒绝 */
} arm_ctrl_t;

extern arm_ctrl_t arm_ctrl;
extern arm_teach_point_t arm_teach_points[ARM_TEACH_MAX_POINTS];
extern uint16_t arm_teach_count;
extern volatile uint8_t arm_busy;

/* 任务 */
void arm_task(void const *argu);
void arm_task_init(void);

/* arm_ik.c 仍在用这个打印宏 */
#ifndef ARM_DBG
extern int dbg_print(const char *fmt, ...);
#define ARM_DBG(fmt, ...)   dbg_print(fmt, ##__VA_ARGS__)
#endif

/* 模式门控：只有 REMOTER 和 ARM_TEACH 允许机械臂输出，且底盘没在跑 */
uint8_t     arm_motion_allowed(void);
const char *arm_mode_name(void);
int  arm_mode_manual(void);        /* 控制台 mode manual */
int  arm_chassis_release(void);    /* 控制台 chassis line：放开底盘，锁住机械臂命令 */
void arm_chassis_stop(void);       /* 控制台 chassis stop */

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
int  arm_flash_commit(void);       /* 把当前 RAM 轨迹和标定写进片内 Flash，0=成功 */
const char *arm_teach_state_name(void);
uint8_t arm_teach_is_recording(void);

/* 死程序采摘 / 视觉逆解：控制台入口，最多带两个字符串参数 */
void arm_pick_cmd(const char *sub, const char *a1, const char *a2);
void arm_ik_cmd(const char *sub, const char *a1, const char *a2);
void arm_pick_toggle(void);        /* 三击：开始 / 停止死程序 */
int  arm_pick_set(uint8_t n, uint16_t shoulder, uint16_t elbow);   /* 控制台 wp：按数字设路点，不读舵机 */

#endif /* __ARM_TASK_H__ */
