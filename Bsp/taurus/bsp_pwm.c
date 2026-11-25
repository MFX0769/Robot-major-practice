/**
  * @file bsp_pwm.c
  * @brief 机械臂夹爪 PWM 输出
  * @note 仅夹爪使用 TIM1_CH3(PE13), 50Hz / 20ms。
  *       肩/肘已改串口总线, 不再占用 TIM1_CH3 以外的通道。
 */
#include "bsp_pwm.h"
#include "tim.h"

void bsp_pwm_init(void)
{
    TIM_OC_InitTypeDef config = {0};

    htim1.Instance = TIM1;
    /* TIM1 时钟 = 240MHz (HSE24*40/2/2)，240 分频 -> 1MHz */
    htim1.Init.Prescaler = 239;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = PWM_PERIOD_US - 1U;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Handler();

    config.OCMode = TIM_OCMODE_PWM1;
    config.Pulse = SERVO_PULSE_ZERO;
    config.OCPolarity = TIM_OCPOLARITY_HIGH;
    config.OCFastMode = TIM_OCFAST_DISABLE;
    config.OCIdleState = TIM_OCIDLESTATE_RESET;
    config.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &config, SERVO_GRIPPER_CHANNEL) != HAL_OK) Error_Handler();
    HAL_TIM_MspPostInit(&htim1);
}

void bsp_pwm_start(void)
{
    HAL_TIM_PWM_Start(&htim1, SERVO_GRIPPER_CHANNEL);
}

void bsp_pwm_stop(void)
{
    HAL_TIM_PWM_Stop(&htim1, SERVO_GRIPPER_CHANNEL);
}

void bsp_pwm_set_pulse(uint8_t servo_id, uint16_t pulse_us)
{
    if (servo_id != SERVO_GRIPPER) return;
    if (pulse_us < SERVO_PULSE_MIN) pulse_us = SERVO_PULSE_MIN;
    if (pulse_us > SERVO_PULSE_MAX) pulse_us = SERVO_PULSE_MAX;
    __HAL_TIM_SET_COMPARE(&htim1, SERVO_GRIPPER_CHANNEL, pulse_us);
}
