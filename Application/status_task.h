#ifndef _STATUS_TASK_H_
#define _STATUS_TASK_H_

#include "stm32h7xx.h"
#include "cmsis_os.h"

#include "stdint.h"

/* 任务状态位 */
#define WDG_BIT_TASK_MODES          (uint32_t)(1 << 0)
#define WDG_BIT_TASK_GIMBAL         (uint32_t)(1 << 1)
#define WDG_BIT_TASK_SHOOT          (uint32_t)(1 << 2)
#define WDG_BIT_TASK_JUDGE          (uint32_t)(1 << 3)
#define WDG_BIT_TASK_DEBUG          (uint32_t)(1 << 4)

/* 硬件通信状态位 */

#define WDG_BIT_BSP_FRIC_MOTO1   		(uint32_t)(1 <<  5)
#define WDG_BIT_BSP_FRIC_MOTO2   		(uint32_t)(1 <<  6)

#define WDG_BIT_BSP_GIMBAL_YAW      (uint32_t)(1 <<  7)
#define WDG_BIT_BSP_GIMBAL_PIT      (uint32_t)(1 <<  8)

#define WDG_BIT_BSP_IMU_ANGLE       (uint32_t)(1 <<  9)
#define WDG_BIT_BSP_IMU_SPEED       (uint32_t)(1 << 10)

#define WDG_BIT_BSP_TRIGGER         (uint32_t)(1 << 11)

#define WDG_BIT_BSP_REMOTE          (uint32_t)(1 << 12)

#define WDG_BIT_BSP_JUDGE           (uint32_t)(1 << 13)


typedef __packed struct {
    uint16_t remote     :1;
    uint16_t gimbal     :1;
    uint16_t imu        :1;
		uint16_t stir       :1;
		uint16_t fric       :1;
    uint16_t judge      :1;
    uint16_t rsvd       :10;
} status_t;

extern status_t status;

extern void wdg_user_set_bit(uint32_t status_bit);
void status_task(void const *argu);


#endif
