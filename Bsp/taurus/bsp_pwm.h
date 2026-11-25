/**
  * @file    bsp_pwm.h
  * @brief   PWM输出封装 (舵机控制用)
  */

#ifndef __BSP_PWM_H__
#define __BSP_PWM_H__

#include "stm32h7xx_hal.h"
#include "prot_servo.h"

/* PWM通道定义 */
/* 夹爪: TIM1_CH3(PE13), 与达妙基座 TIM1 引脚一致 */
#define SERVO_GRIPPER_TIM         TIM1
#define SERVO_GRIPPER_CHANNEL     TIM_CHANNEL_3   /* PE13 */

/* PWM参数: TIM1 时钟 240MHz, 240 分频 -> 1MHz */
#define PWM_TIM_FREQ           1000000  /* 1MHz计数频率 */
#define PWM_PERIOD_US          20000    /* 20ms周期 */

/* 函数声明 */
void bsp_pwm_init(void);
void bsp_pwm_set_pulse(uint8_t channel, uint16_t pulse_us);
void bsp_pwm_start(void);
void bsp_pwm_stop(void);

#endif
