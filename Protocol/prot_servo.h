/**
  * @file    prot_servo.h
  * @brief   PWM舵机驱动接口
  * @note    PWM舵机方案，非总线/串口舵机
  */

#ifndef __PROT_SERVO_H__
#define __PROT_SERVO_H__

#include <stdint.h>

/* 舵机数量 */
#define SERVO_NUM          3    /* 肩、肘、夹爪 (yaw用GM6020) */

/* PWM参数 (单位: us) */
#define SERVO_PULSE_MIN    500
#define SERVO_PULSE_MAX    2500
#define SERVO_PULSE_ZERO   1500
#define SERVO_PERIOD       20000  /* 20ms */

/* 舵机ID */
typedef enum {
    SERVO_SHOULDER = 0,  /* 肩关节 */
    SERVO_ELBOW    = 1,  /* 肘关节 */
    SERVO_GRIPPER  = 2,  /* 夹爪 */
} servo_id_e;

/* 舵机参数结构体 (每个关节独立标定) */
typedef struct {
    uint16_t pulse_min;      /* 最小脉宽 (us) */
    uint16_t pulse_max;      /* 最大脉宽 (us) */
    uint16_t pulse_zero;     /* 零位脉宽 (us) */
    int8_t   direction;      /* 方向: 1或-1 */
    float    angle_min;      /* 最小角度 (度) */
    float    angle_max;      /* 最大角度 (度) */
    uint16_t current_pulse;  /* 当前脉宽 (调试用) */
} servo_param_t;

/* 全局变量 */
extern servo_param_t servo_param[SERVO_NUM];

/* 函数声明 */
void servo_init(void);
void servo_enable(uint8_t id);
void servo_disable(uint8_t id);
void servo_set_angle(uint8_t id, float angle_deg);
void servo_set_pulse(uint8_t id, uint16_t pulse_us);
float servo_angle_to_pulse(uint8_t id, float angle_deg);
uint16_t servo_pulse_limit(uint8_t id, uint16_t pulse);

/* 调试和监控函数 */
void servo_debug_info(void);
void servo_print_params(void);
uint16_t servo_get_current_pulse(uint8_t id);

#endif
