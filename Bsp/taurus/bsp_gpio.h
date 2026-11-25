/**
  * @file    bsp_gpio.h
  * @brief   GPIO封装 (舵机供电使能控制)
  * @note    单路总电源控制，通过MOS管开关舵机供电
  */

#ifndef __BSP_GPIO_H__
#define __BSP_GPIO_H__

#include "main.h"

/* Includes ------------------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* 达妙 MC-Board02 可控 5V 使能: PC15, 高电平开 (原理图 Power_5V_EN) */
#define SERVO_POWER_PORT       POWER_5V_GPIO_Port
#define SERVO_POWER_PIN        POWER_5V_Pin

/* 供电控制极性: 1=高电平使能, 0=低电平使能 */
#define SERVO_POWER_ACTIVE     1

/* Exported function declarations --------------------------------------------*/

void bsp_gpio_init(void);
void bsp_gpio_servo_power_on(void);
void bsp_gpio_servo_power_off(void);

#endif /* __BSP_GPIO_H__ */
