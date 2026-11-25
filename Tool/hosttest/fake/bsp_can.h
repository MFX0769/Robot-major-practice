#ifndef FAKE_BSP_CAN_H
#define FAKE_BSP_CAN_H
#include <stdint.h>

typedef enum { CAN_CHANNEL_1 = 0, CAN_CHANNEL_2 = 1 } can_channel_e;

/* 与真头文件字段一致（真文件依赖 HAL 的 FDCAN 类型） */
typedef struct
{
    uint32_t tx_ok;
    uint32_t rx;
    uint32_t bus_off;
    uint32_t recovered;
    uint8_t  tec;
    uint8_t  rec;
    uint8_t  bo;
    uint8_t  lec;
} bsp_can_stat_t;

void bsp_can_stat_get(can_channel_e ch, bsp_can_stat_t *out);
void bsp_can_recover(can_channel_e ch);
#endif
