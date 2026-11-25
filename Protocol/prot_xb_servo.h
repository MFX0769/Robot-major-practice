/**
  * @file prot_xb_servo.h
  * @brief 星呗 SP 系列单线半双工总线舵机驱动
  * @note 硬件绑定: USART1(PA9) 115200-8N1, HDSEL 单线半双工。
  *       肩/肘使用总线舵机; 接收 ISR 只入环形缓冲, arm_task 周期 poll。
 */
#ifndef PROT_XB_SERVO_H
#define PROT_XB_SERVO_H

#include <stdint.h>

#define XB_BAUDRATE        115200U
#define XB_JOINT_NUM       2U
#define XB_PULSE_MIN       500U
#define XB_PULSE_MAX       2500U
#define XB_PULSE_ZERO      1500U
#define XB_TIME_MAX_MS     9999U
#define XB_ONLINE_TIMEOUT  500U

typedef enum {
    XB_SHOULDER = 0,
    XB_ELBOW    = 1,
} xb_joint_e;

typedef struct {
    uint8_t  id;
    uint8_t  online;
    uint16_t pulse;
    int16_t  temp_c;
    int16_t  volt_v;
    uint32_t last_ok_tick;
} xb_servo_dev_t;

extern xb_servo_dev_t xb_servo[XB_JOINT_NUM];

void xb_servo_init(void);
uint8_t xb_servo_set_pos(uint8_t joint, uint16_t pulse, uint16_t time_ms);
uint8_t xb_servo_release(uint8_t joint);
uint8_t xb_servo_lock(uint8_t joint);
uint8_t xb_servo_read_pos(uint8_t joint, uint16_t *pulse);
uint8_t xb_servo_read_id(uint8_t joint, uint8_t *id);
uint8_t xb_servo_read_id_raw(uint8_t servo_id, uint8_t *id);
uint8_t xb_servo_write_id_raw(uint8_t old_id, uint8_t new_id);
uint8_t xb_servo_ping_raw(uint8_t id);
const char *xb_servo_last_frame(void);
void xb_servo_poll(void);
uint16_t xb_servo_get_pulse(uint8_t joint);
uint8_t xb_servo_online(uint8_t joint);
uint8_t xb_servo_bind(uint8_t joint, uint8_t id);
uint32_t xb_servo_rx_errors(void);
void xb_servo_rx_isr(void);

#endif
