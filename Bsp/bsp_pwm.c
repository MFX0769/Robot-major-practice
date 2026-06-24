/**
  * @file    bsp_pwm.c
  * @brief   PWM输出实现 (双定时器)
  * @note    TIM2: 肩关节(PA0) + 肘关节(PA1)
  *          TIM3: 夹爪(PA6)
  */

#include "bsp_pwm.h"
#include "main.h"

TIM_HandleTypeDef htim3;

/**
  * @brief  PWM定时器初始化
  */
void bsp_pwm_init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};
    
    /* ============ TIM2: 肩关节 + 肘关节 ============ */
    __HAL_RCC_TIM2_CLK_ENABLE();
    
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 83;  /* 84MHz/84 = 1MHz */
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 19999;  /* 1MHz/20000 = 50Hz (20ms) */
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.RepetitionCounter = 0;
    HAL_TIM_PWM_Init(&htim2);
    
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 1500;  /* 默认1500us中位 */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, SERVO_SHOULDER_CHANNEL);
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, SERVO_ELBOW_CHANNEL);
    
    /* ============ TIM3: 夹爪 ============ */
    __HAL_RCC_TIM3_CLK_ENABLE();
    
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 83;  /* 84MHz/84 = 1MHz */
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 19999;  /* 1MHz/20000 = 50Hz (20ms) */
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.RepetitionCounter = 0;
    HAL_TIM_PWM_Init(&htim3);
    
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, SERVO_GRIPPER_CHANNEL);
}

/**
  * @brief  启动PWM输出
  */
void bsp_pwm_start(void)
{
    HAL_TIM_PWM_Start(&htim2, SERVO_SHOULDER_CHANNEL);
    HAL_TIM_PWM_Start(&htim2, SERVO_ELBOW_CHANNEL);
    HAL_TIM_PWM_Start(&htim3, SERVO_GRIPPER_CHANNEL);
}

/**
  * @brief  停止PWM输出
  */
void bsp_pwm_stop(void)
{
    HAL_TIM_PWM_Stop(&htim2, SERVO_SHOULDER_CHANNEL);
    HAL_TIM_PWM_Stop(&htim2, SERVO_ELBOW_CHANNEL);
    HAL_TIM_PWM_Stop(&htim3, SERVO_GRIPPER_CHANNEL);
}

/**
  * @brief  设置指定舵机脉宽
  * @param  servo_id: 0=肩关节, 1=肘关节, 2=夹爪
  * @param  pulse_us: 脉宽 (us)
  */
void bsp_pwm_set_pulse(uint8_t servo_id, uint16_t pulse_us)
{
    /* 限制脉宽范围 */
    if (pulse_us < 500) pulse_us = 500;
    if (pulse_us > 2500) pulse_us = 2500;
    
    switch (servo_id) {
        case 0:  /* 肩关节 - TIM2_CH1 */
            __HAL_TIM_SET_COMPARE(&htim2, SERVO_SHOULDER_CHANNEL, pulse_us);
            break;
        case 1:  /* 肘关节 - TIM2_CH2 */
            __HAL_TIM_SET_COMPARE(&htim2, SERVO_ELBOW_CHANNEL, pulse_us);
            break;
        case 2:  /* 夹爪 - TIM3_CH1 */
            __HAL_TIM_SET_COMPARE(&htim3, SERVO_GRIPPER_CHANNEL, pulse_us);
            break;
        default:
            break;
    }
}
