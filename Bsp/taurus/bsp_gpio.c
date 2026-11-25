/**
  * @file    bsp_gpio.c
  * @brief   GPIO实现 (舵机供电使能控制)
  * @note    单路总电源控制，通过NMOS开关舵机供电
  *          PC15 -> Power_5V_EN,高电平使能
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_gpio.h"

/* Private function prototypes -----------------------------------------------*/

/* ========================= GPIO初始化 ========================= */
void bsp_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 使能GPIOA时钟 */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* 配置PC15为推挽输出 */
    GPIO_InitStruct.Pin   = SERVO_POWER_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SERVO_POWER_PORT, &GPIO_InitStruct);

    /* 2026-11-25 用户要求：舵机已接，5V 上电默认使能（达妙例程同样默认 SET）。 */
    bsp_gpio_servo_power_on();
}

/* ========================= 使能舵机供电 ========================= */
void bsp_gpio_servo_power_on(void)
{
#if SERVO_POWER_ACTIVE
    HAL_GPIO_WritePin(SERVO_POWER_PORT, SERVO_POWER_PIN, GPIO_PIN_SET);
#else
    HAL_GPIO_WritePin(SERVO_POWER_PORT, SERVO_POWER_PIN, GPIO_PIN_RESET);
#endif
}

/* ========================= 关闭舵机供电 ========================= */
void bsp_gpio_servo_power_off(void)
{
#if SERVO_POWER_ACTIVE
    HAL_GPIO_WritePin(SERVO_POWER_PORT, SERVO_POWER_PIN, GPIO_PIN_RESET);
#else
    HAL_GPIO_WritePin(SERVO_POWER_PORT, SERVO_POWER_PIN, GPIO_PIN_SET);
#endif
}
