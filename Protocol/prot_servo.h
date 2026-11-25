/**
  * @file prot_servo.h
  * @brief 机械臂舵机统一接口
  * @note 肩/肘为 SP 系列串口总线舵机, 夹爪为 PWM 舵机。
  *       业务层只使用本接口, 不直接操作 UART 或定时器。
 */
#ifndef __PROT_SERVO_H__
#define __PROT_SERVO_H__

#include <stdint.h>

#define SERVO_NUM          3U
#define SERVO_PULSE_MIN    500U
#define SERVO_PULSE_MAX    2500U
#define SERVO_PULSE_ZERO   1500U
#define SERVO_PERIOD       20000U

typedef enum {
    SERVO_SHOULDER = 0,
    SERVO_ELBOW    = 1,
    SERVO_GRIPPER  = 2,
} servo_id_e;

typedef struct {
    uint16_t pulse_min;
    uint16_t pulse_max;
    uint16_t pulse_zero;
    int8_t   direction;
    float    angle_min;
    float    angle_max;
    uint16_t current_pulse;
    uint16_t target_pulse;
} servo_param_t;

extern servo_param_t servo_param[SERVO_NUM];

void servo_init(void);
void servo_poll(void);
void servo_set_enabled(uint8_t enabled);
void servo_set_config_mode(uint8_t enabled);
uint8_t servo_config_active(void);
void servo_set_angle(uint8_t id, float angle_deg);
void servo_set_pulse(uint8_t id, uint16_t pulse_us);
void servo_set_pulse_time(uint8_t id, uint16_t pulse_us, uint16_t time_ms);
float servo_angle_to_pulse(uint8_t id, float angle_deg);
uint16_t servo_pulse_limit(uint8_t id, uint16_t pulse);
void servo_debug_info(void);
void servo_print_params(void);
uint16_t servo_get_current_pulse(uint8_t id);
uint8_t servo_online(uint8_t id);
#define SERVO_TEACH_CAPACITY 64U

typedef struct {
    uint16_t shoulder;
    uint16_t elbow;
} servo_teach_point_t;

extern servo_teach_point_t servo_teach_points[SERVO_TEACH_CAPACITY];
extern uint16_t servo_teach_count;

uint8_t servo_teach_begin(void);
void servo_teach_poll(void);
void servo_teach_end(void);
uint8_t servo_teach_active(void);

#endif
