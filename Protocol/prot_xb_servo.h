/**
  * @file    prot_xb_servo.h
  * @brief   星呗(xbrobot) SP 系列单线总线舵机驱动接口
  * @note    状态：待激活。车上肩/肘/夹爪舵机疑似 SP-30S 总线舵机（单线半双工
  *          UART 115200-8N1 文本协议），但尚未做决定性实验确认（USB-TTL 发
  *          #000PVER! 看是否有 #...G! 应答，5 分钟）。实验通过后：
  *          1) arm_task.c 里 #include 本头文件并用本驱动替换 servo_set_pulse()；
  *          2) 保留 prot_servo(PWM)/bsp_pwm 作保底不删。
  *          协议全表见 Develop_Log/20261120_机械臂架构与舵机通信调研.md。
  *          开源参照：Chenxi-Li24/OmniX_Engineer（RMUL2026 上海站冠军）。
  */

#ifndef __PROT_XB_SERVO_H__
#define __PROT_XB_SERVO_H__

#include <stdint.h>

/* ---- 总线硬件绑定（换引脚只改这里） ---- */
#define XB_BAUDRATE        115200   /* 手册默认，可用 #xxxPBDn! 改 */
#define XB_JOINT_NUM       3        /* 肩 / 肘 / 夹爪 */

/* ---- 协议常量（串行总线舵机指令表.pdf） ---- */
#define XB_PULSE_MIN       500      /* P 4 位补零 0500~2500 对应 0~270° */
#define XB_PULSE_MAX       2500
#define XB_PULSE_ZERO      1500
#define XB_TIME_MAX_MS     9999     /* T 4 位补零 */
#define XB_BROADCAST_ID    255      /* #255xx! 广播（慎用：多舵机同收） */

/* 关节编号（上层只认关节，不认舵机 ID；ID 在设备表里配） */
typedef enum {
    XB_SHOULDER = 0,   /* 肩关节 */
    XB_ELBOW    = 1,   /* 肘关节 */
    XB_GRIPPER  = 2,   /* 夹爪 */
} xb_joint_e;

/* 单个舵机设备状态（poll 周期更新） */
typedef struct {
    uint8_t  id;          /* 总线 ID，默认 0/1/2，现场可用 #xxxPIDyyy! 重分配 */
    uint8_t  online;      /* 最近一次 PRAD 应答有效 */
    uint16_t pulse;       /* 最近回读脉宽（PULK 释力示教时 = 手扳位置） */
    int16_t  temp_c;      /* PRTV 回读温度 ℃ */
    int16_t  volt_v;      /* PRTV 回读电压 V */
} xb_servo_dev_t;

extern xb_servo_dev_t xb_servo[XB_JOINT_NUM];

/* ---- 接口 ---- */
void    xb_servo_init(void);                                 /* UART4 单线半双工初始化(PC10 AF8) */
uint8_t xb_servo_set_pos(uint8_t joint, uint16_t pulse, uint16_t time_ms);  /* 控制帧 #xxxPxxxxTxxxx! */
uint8_t xb_servo_release(uint8_t joint);                     /* PULK 释力（示教/保护用） */
uint8_t xb_servo_request_pos(uint8_t joint);                 /* 发 PRAD 读位置请求 */
void    xb_servo_poll(void);                                 /* 周期调用：解析应答帧更新设备表 */
uint16_t xb_servo_get_pulse(uint8_t joint);                  /* 最近回读脉宽 */
uint8_t xb_servo_ping(uint8_t joint);                        /* PVER 探测，1=有应答 */

/* 返回值：0=OK  1=参数越界  2=忙/请求在途 */
#endif
