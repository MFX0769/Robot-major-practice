/**
  * @file    prot_servo.c
  * @brief   PWM舵机驱动实现
  * @note    PWM舵机方案，非总线/串口舵机
  *          标定变量保持易于Keil Debug修改
  */

#include "prot_servo.h"
#include "bsp_pwm.h"
#include "bsp_gpio.h"

/* 舵机参数 (标定用，可在Keil Debug中修改) */
servo_param_t servo_param[SERVO_NUM] = {
    /* 肩关节: pulse_min=500, pulse_max=2500, zero=1500, dir=1, angle: -15~38(实测机械限位，2026-06-25) */
    {500, 2500, 1500, 1, -15.0f, 38.0f, 1500},
    /* 肘关节: pulse_min=500, pulse_max=2500, zero=1500, dir=1, angle: -5~51(实测机械限位，2026-06-25) */
    {500, 2500, 1500, 1, -5.0f, 51.0f, 1500},
    /* 夹爪: pulse_min=500, pulse_max=2500, zero=1500, dir=1, angle: -49~-7(实测机械限位，张开~闭合，2026-06-25) */
    {500, 2500, 1500, 1, -49.0f, -7.0f, 1500},
};

/**
  * @brief  舵机初始化
  */
void servo_init(void)
{
    /* 初始化PWM输出 */
    bsp_pwm_init();
    
    /* 初始化供电使能GPIO */
    bsp_gpio_init();
    
    /* 默认关闭舵机供电 */
    bsp_gpio_servo_power_off();
}

/**
  * @brief  使能舵机供电
  * @param  id: 舵机ID
  */
void servo_enable(uint8_t id)
{
    (void)id;  /* 单路总电源，忽略channel参数 */
    bsp_gpio_servo_power_on();
}

/**
  * @brief  失能舵机供电
  * @param  id: 舵机ID
  */
void servo_disable(uint8_t id)
{
    (void)id;  /* 单路总电源，忽略channel参数 */
    bsp_gpio_servo_power_off();
}

/**
  * @brief  角度转脉宽
  * @param  id: 舵机ID
  * @param  angle_deg: 目标角度 (度)
  * @retval 脉宽 (us)
  */
float servo_angle_to_pulse(uint8_t id, float angle_deg)
{
    if (id >= SERVO_NUM) return SERVO_PULSE_ZERO;
    
    servo_param_t *p = &servo_param[id];
    
    /* 线性映射: angle → pulse */
    float range = (float)(p->pulse_max - p->pulse_min);
    float angle_range = p->angle_max - p->angle_min;
    
    if (angle_range < 0.001f) return p->pulse_zero;
    
    float normalized = (angle_deg - p->angle_min) / angle_range;
    float pulse = (float)p->pulse_min + normalized * range;
    
    /* 方向修正 */
    if (p->direction < 0) {
        pulse = (float)p->pulse_max - (pulse - (float)p->pulse_min);
    }
    
    return pulse;
}

/**
  * @brief  脉宽限幅
  * @param  id: 舵机ID
  * @param  pulse: 输入脉宽 (us)
  * @retval 限幅后的脉宽 (us)
  */
uint16_t servo_pulse_limit(uint8_t id, uint16_t pulse)
{
    if (id >= SERVO_NUM) return SERVO_PULSE_ZERO;
    
    servo_param_t *p = &servo_param[id];
    
    if (pulse < p->pulse_min) pulse = p->pulse_min;
    if (pulse > p->pulse_max) pulse = p->pulse_max;
    
    return pulse;
}

/**
  * @brief  设置舵机角度
  * @param  id: 舵机ID
  * @param  angle_deg: 目标角度 (度)
  */
void servo_set_angle(uint8_t id, float angle_deg)
{
    if (id >= SERVO_NUM) return;
    
    /* 角度→脉宽 */
    float pulse_f = servo_angle_to_pulse(id, angle_deg);
    uint16_t pulse = (uint16_t)(pulse_f + 0.5f);
    
    /* 脉宽限幅 */
    pulse = servo_pulse_limit(id, pulse);
    
    /* 更新当前值 */
    servo_param[id].current_pulse = pulse;
    
    /* 输出PWM */
    bsp_pwm_set_pulse(id, pulse);
}

/**
  * @brief  直接设置脉宽 (调试用)
  * @param  id: 舵机ID
  * @param  pulse_us: 脉宽 (us)
  */
void servo_set_pulse(uint8_t id, uint16_t pulse_us)
{
    if (id >= SERVO_NUM) return;
    
    /* 脉宽限幅 */
    pulse_us = servo_pulse_limit(id, pulse_us);
    
    /* 更新当前值 */
    servo_param[id].current_pulse = pulse_us;
    
    /* 输出PWM */
    bsp_pwm_set_pulse(id, pulse_us);
}

/**
  * @brief  获取舵机当前脉宽 (调试用)
  * @param  id: 舵机ID
  * @retval 当前脉宽 (us)
  */
uint16_t servo_get_current_pulse(uint8_t id)
{
    if (id >= SERVO_NUM) return 0;
    return servo_param[id].current_pulse;
}

/**
  * @brief  打印舵机参数 (调试用)
  */
void servo_print_params(void)
{
    for (uint8_t i = 0; i < SERVO_NUM; i++) {
        /* TODO: 添加串口打印输出 */
        /* printf("Servo%d: pulse_min=%d, pulse_max=%d, angle_min=%.1f, angle_max=%.1f, current_pulse=%d\n",
               i, servo_param[i].pulse_min, servo_param[i].pulse_max, 
               servo_param[i].angle_min, servo_param[i].angle_max, servo_param[i].current_pulse); */
    }
}

/**
  * @brief  输出舵机调试信息
  */
void servo_debug_info(void)
{
    static uint32_t debug_cnt = 0;
    if (debug_cnt++ % 200 == 0) {
        servo_print_params();
    }
}
