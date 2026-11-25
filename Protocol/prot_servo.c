/**
  * @file prot_servo.c
  * @brief 机械臂舵机统一驱动
  * @note 肩/肘走 SP 串口总线, 夹爪走 PWM; 保留原 servo_* 业务接口。
 */
#include "prot_servo.h"
#include "prot_xb_servo.h"
#include "bsp_pwm.h"
#include "bsp_gpio.h"
#include "servo_test.h"
#include "prot_dr16.h"
#include "cmsis_os.h"

servo_param_t servo_param[SERVO_NUM] = {
    {500, 2500, 1500, 1, -15.0f, 38.0f, 1500, 1500},
    {500, 2500, 1500, 1, -5.0f, 51.0f, 1500, 1500},
    {500, 2500, 1500, 1, -49.0f, -7.0f, 1500, 1500},
};

static uint8_t s_enabled;
static uint8_t s_teach_active;
static uint32_t s_teach_last_tick;
servo_teach_point_t servo_teach_points[SERVO_TEACH_CAPACITY];
uint16_t servo_teach_count;

void servo_init(void)
{
    bsp_pwm_init();
    bsp_gpio_init();
    xb_servo_init();
    bsp_pwm_start();
    servo_test_init();
    servo_set_enabled(1);   /* 舵机已接，5V 保持使能 */
}

void servo_poll(void)
{
    servo_test_process();
    static uint8_t feedback_div;
    static uint8_t feedback_joint;
    uint16_t pulse;

    xb_servo_poll();
    if (!s_enabled) return;
    if (++feedback_div >= 34U) {
        feedback_div = 0;
        if (xb_servo_read_pos(feedback_joint, &pulse) == 0U) {
            servo_param[feedback_joint].current_pulse = pulse;
        }
        feedback_joint ^= 1U;
    }
    for (uint8_t i = 0; i < XB_JOINT_NUM; i++) {
        if (xb_servo_online(i)) servo_param[i].current_pulse = xb_servo_get_pulse(i);
    }
}

void servo_set_enabled(uint8_t enabled)
{
    if (enabled) bsp_gpio_servo_power_on();
    else bsp_gpio_servo_power_off();
    s_enabled = enabled ? 1U : 0U;
}

void servo_set_config_mode(uint8_t enabled)
{
    if (enabled) {
        servo_set_enabled(1);
        servo_test_set_config_allowed(1);
    } else {
        servo_test_set_config_allowed(0);
        /* 退出配置模式不回断 5V：舵机保持使能，避免 servo run 后失电 */
        servo_set_enabled(1);
    }
}

uint8_t servo_config_active(void)
{
    return servo_test_config_active();
}

float servo_angle_to_pulse(uint8_t id, float angle_deg)
{
    if (id >= SERVO_NUM) return SERVO_PULSE_ZERO;
    servo_param_t *p = &servo_param[id];
    float range = (float)(p->pulse_max - p->pulse_min);
    float angle_range = p->angle_max - p->angle_min;
    if (angle_range < 0.001f) return p->pulse_zero;

    float normalized = (angle_deg - p->angle_min) / angle_range;
    float pulse = (float)p->pulse_min + normalized * range;
    if (p->direction < 0) pulse = (float)p->pulse_max - (pulse - (float)p->pulse_min);
    return pulse;
}

uint16_t servo_pulse_limit(uint8_t id, uint16_t pulse)
{
    if (id >= SERVO_NUM) return SERVO_PULSE_ZERO;
    if (pulse < servo_param[id].pulse_min) pulse = servo_param[id].pulse_min;
    if (pulse > servo_param[id].pulse_max) pulse = servo_param[id].pulse_max;
    return pulse;
}

void servo_set_angle(uint8_t id, float angle_deg)
{
    if (id >= SERVO_NUM) return;
    uint16_t pulse = (uint16_t)(servo_angle_to_pulse(id, angle_deg) + 0.5f);
    servo_set_pulse(id, pulse);
}

void servo_set_pulse(uint8_t id, uint16_t pulse_us)
{
    if (id >= SERVO_NUM) return;
    pulse_us = servo_pulse_limit(id, pulse_us);

    if (id < XB_JOINT_NUM) {
        if (servo_param[id].target_pulse != pulse_us) {
            servo_param[id].target_pulse = pulse_us;
            /* 3ms 任务周期, T=20ms 让舵机内部完成平滑运动。 */
            (void)xb_servo_set_pos(id, pulse_us, 20U);
        }
    } else {
        bsp_pwm_set_pulse(id, pulse_us);
    }
    servo_param[id].current_pulse = pulse_us;
}

uint16_t servo_get_current_pulse(uint8_t id)
{
    return id < SERVO_NUM ? servo_param[id].current_pulse : 0U;
}

uint8_t servo_teach_begin(void)
{
    uint16_t shoulder, elbow;
    if (s_teach_active) return 0U;
    if (xb_servo_read_pos(SERVO_SHOULDER, &shoulder) != 0U ||
        xb_servo_read_pos(SERVO_ELBOW, &elbow) != 0U) return 1U;
    servo_set_enabled(1);
    s_teach_active = 1U;
    s_teach_last_tick = osKernelSysTick();
    servo_teach_count = 0;
    {
        uint16_t shoulder, elbow;
        if (xb_servo_read_pos(SERVO_SHOULDER, &shoulder) == 0U) servo_param[SERVO_SHOULDER].current_pulse = shoulder;
        if (xb_servo_read_pos(SERVO_ELBOW, &elbow) == 0U) servo_param[SERVO_ELBOW].current_pulse = elbow;
    }
    uint8_t rc = xb_servo_release(SERVO_SHOULDER);
    if (rc != 0U || xb_servo_release(SERVO_ELBOW) != 0U) {
        (void)xb_servo_lock(SERVO_SHOULDER);
        (void)xb_servo_lock(SERVO_ELBOW);
        s_teach_active = 0U;
        return 2U;
    }
    return 0U;
}

void servo_teach_poll(void)
{
    if (!s_teach_active) return;
    if ((uint32_t)(osKernelSysTick() - s_teach_last_tick) < 100U) return;
    s_teach_last_tick = osKernelSysTick();
    uint16_t shoulder = servo_param[SERVO_SHOULDER].current_pulse;
    uint16_t elbow = servo_param[SERVO_ELBOW].current_pulse;
    if (xb_servo_read_pos(SERVO_SHOULDER, &shoulder) != 0U ||
        xb_servo_read_pos(SERVO_ELBOW, &elbow) != 0U) return;
    if (servo_teach_count < SERVO_TEACH_CAPACITY) {
        servo_teach_points[servo_teach_count].shoulder = shoulder;
        servo_teach_points[servo_teach_count].elbow = elbow;
        servo_teach_count++;
    }
}

void servo_teach_end(void)
{
    if (!s_teach_active) return;
    (void)xb_servo_lock(SERVO_SHOULDER);
    (void)xb_servo_lock(SERVO_ELBOW);
    s_teach_active = 0U;
}

uint8_t servo_teach_active(void)
{
    return s_teach_active;
}

uint8_t servo_online(uint8_t id)
{
    return id < XB_JOINT_NUM ? xb_servo_online(id) : 1U;
}

void servo_print_params(void)
{
    /* 参数通过调试器变量表维护, 启动时不产生额外串口流量。 */
}

void servo_debug_info(void)
{
    servo_print_params();
}
