/**
  * @file    bsp_gpio.h
  * @brief   GPIO封装 (舵机供电使能控制)
  * @note    单路总电源控制，通过MOS管开关舵机供电
  */

#ifndef __BSP_GPIO_H__
#define __BSP_GPIO_H__

#include "stm32h7xx_hal.h"

/* Includes ------------------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* 舵机供电使能引脚 (PA7, 通过NMOS控制总电源) */
#define SERVO_POWER_PORT       GPIOA
#define SERVO_POWER_PIN        GPIO_PIN_7

/* 供电控制极性: 1=高电平使能, 0=低电平使能 */
#define SERVO_POWER_ACTIVE     1

/* Exported function declarations --------------------------------------------*/

void bsp_gpio_init(void);
void bsp_gpio_servo_power_on(void);
void bsp_gpio_servo_power_off(void);

#endif /* __BSP_GPIO_H__ */
