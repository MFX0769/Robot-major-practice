/* 主机测试台的模拟硬件接口。固件源文件不改，只把这些换掉。 */
#ifndef SIM_H
#define SIM_H

#include <stdint.h>

extern uint32_t g_now_ms;          /* 模拟的毫秒时钟 */
extern int      g_verbose;         /* 1 = 打印固件日志 */

/* 按键：1 = 按下（低电平） */
extern int      g_btn_pressed;

/* 总线舵机（肩 0 / 肘 1） */
extern int16_t  g_pulse[2];        /* 真实位置 */
extern uint8_t  g_released;        /* 1 = 释力，可以用手拖 */
extern uint8_t  g_servo_online[2];
extern uint16_t g_gripper_cmd;     /* 夹爪最近一次命令 */
extern uint8_t  g_config_active;   /* servo config 模式 */

/* 6020 */
extern int      g_yaw_online;
extern int      g_yaw_wiring_dir;  /* +1: 正电压让 ECD 增大；-1: 线接反 */
extern int16_t  g_yaw_tx;          /* 最近一帧电压 */
extern float    g_yaw_angle;       /* ECD，0~8192 连续 */
extern uint32_t g_yaw_frames;

/* 舵机命令记录 */
typedef struct {
    uint32_t t;                    /* 发出时刻 */
    uint8_t  joint;                /* 0 肩 1 肘 2 夹爪 */
    uint16_t pulse;
    uint16_t time_ms;
} cmd_log_t;
extern cmd_log_t g_cmd_log[512];
extern uint32_t  g_cmd_log_n;

/* Flash 操作计数、自检结果（0 未见 1 PASS 2 FAIL） */
extern uint32_t g_flash_erase_cnt;
extern uint32_t g_flash_program_cnt;
extern int      g_flag_flash_test;

/* 底盘：4 个 2006 */
extern int16_t  g_motor_tx[4];         /* 最近一帧里 4 个电机的电流给定 */
extern uint32_t g_chassis_tx_frames;   /* 底盘任务发帧次数 */
void   sim_wheel_set_rpm(int idx, int rpm);
void   sim_line_set(uint8_t state, int16_t position);   /* 巡线模块给的 state / position */

/* can raw 记录 */
extern uint32_t g_canraw_cnt;
extern uint32_t g_canraw_ch;
extern uint32_t g_canraw_id;
extern uint32_t g_canraw_n;
extern uint8_t  g_canraw_d[8];

/* 串口接收模型：uart = 5（DBUS）或 10（控制台） */
void   sim_uart_rx_push(int uart, uint8_t byte);
void   sim_uart_rx_idle(int uart);                 /* 一帧发完，总线空闲 */
void   sim_uart_rx_error(int uart, uint32_t flags);

/* osDelayUntil 的替身：底盘测试用它从任务死循环里跳出来 */
typedef int (*sim_delayuntil_fn)(uint32_t *ticks, uint32_t ms);
extern sim_delayuntil_fn g_delayuntil_hook;

/* 固件打印的截取：g_capture = 1 时 dbg_print 的内容攒在 g_out 里，测试用 sim_out_has 找 */
#define G_OUT_CAP 16384
extern char     g_out[G_OUT_CAP];
extern uint32_t g_out_len;
extern int      g_capture;
void sim_out_clear(void);
int  sim_out_has(const char *s);

/* 控制台用的假舵机驱动/巡线/W25Q64 状态（sim_stubs_console.c） */
void sim_dbus_set(uint32_t bytes, uint32_t frames, uint8_t online);
void sim_can_set(int ch, uint32_t tx, uint32_t rx, uint8_t tec, uint8_t bo, uint32_t off_total);
void sim_rc_set_sw2(uint8_t dbus_sw2);
extern uint32_t g_sbus_calls;
extern uint32_t g_w25_erase_calls;
extern uint32_t g_wdg_bits;
extern uint8_t  g_rc_last[18];

void   sim_reset(void);
void   sim_advance(uint32_t ms);                 /* 时间前进，期间舵机运动、yaw 电机和轮组响应 */
void   sim_set_pulse(uint8_t joint, int16_t pulse);   /* 直接摆姿势，相当于手拖 */

#endif
