#ifndef _STATUS_TASK_H_
#define _STATUS_TASK_H_

#include "stm32h7xx.h"
#include "cmsis_os.h"

#include "stdint.h"

/* 任务状态位 */
#define WDG_BIT_TASK_MODES          (uint32_t)(1 << 0)

/* 硬件通信状态位 */
#define WDG_BIT_BSP_IMU_ANGLE       (uint32_t)(1 <<  9)
#define WDG_BIT_BSP_IMU_SPEED       (uint32_t)(1 << 10)
#define WDG_BIT_BSP_REMOTE          (uint32_t)(1 << 12)
#define WDG_BIT_BSP_JUDGE           (uint32_t)(1 << 13)


typedef struct {
    uint16_t remote     :1;
    uint16_t imu        :1;
    uint16_t judge      :1;
    uint16_t rsvd       :13;
} status_t;

extern status_t status;

void wdg_user_set_bit(uint32_t status_bit);
void status_task(void const *argu);


#endif
