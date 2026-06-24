/**
  * @file    bsp_pwm.h
  * @brief   PWM输出封装 (舵机控制用)
  */

#ifndef __BSP_PWM_H__
#define __BSP_PWM_H__

#include "stm32h7xx_hal.h"

/* PWM通道定义 */
/* 肩关节+肘关节: TIM2 */
#define SERVO_SHOULDER_TIM        TIM2
#define SERVO_SHOULDER_CHANNEL    TIM_CHANNEL_1   /* PA0 */
#define SERVO_ELBOW_TIM           TIM2
#define SERVO_ELBOW_CHANNEL       TIM_CHANNEL_2   /* PA1 */

/* 夹爪: TIM3 (独立) */
#define SERVO_GRIPPER_TIM         TIM3
#define SERVO_GRIPPER_CHANNEL     TIM_CHANNEL_1   /* PA6 */

/* PWM参数 (84MHz / 84 = 1MHz, ARR=19999 → 20ms) */
#define PWM_TIM_FREQ           1000000  /* 1MHz计数频率 */
#define PWM_PERIOD_US          20000    /* 20ms周期 */

/* 函数声明 */
void bsp_pwm_init(void);
void bsp_pwm_set_pulse(uint8_t channel, uint16_t pulse_us);
void bsp_pwm_start(void);
void bsp_pwm_stop(void);

#endif
