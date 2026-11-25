/**
  ******************************************************************************
  * @file    bsp_can.h
  * @brief   CAN1 BSP 接口。
  ******************************************************************************
  */
#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "fdcan.h"
#include <stdint.h>

typedef enum
{
    FLT_RANGE = 0x0000U,
    FLT_DUAL  = 0x0001U,
    FLT_MASK  = 0x0002U,
} bsp_can_filter_type_t;

typedef enum
{
    CAN_CHANNEL_1,
    CAN_CHANNEL_2,
    CAN_CHANNEL_NUM,
} can_channel_e;

typedef struct
{
    uint8_t count;
    uint16_t data[30];     /* 最多 10 条，每条 type/id1/id2 三个 u16 */
} bsp_can_filt_tbl_t;

typedef struct
{
    FDCAN_TxHeaderTypeDef txHeader;
    uint8_t tx_data[8];
} CAN_TxBuffer_t;

typedef struct
{
    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t rx_data[8];
} CAN_RxBuffer_t;

typedef struct
{
    FDCAN_HandleTypeDef *hfdcan;
    CAN_TxBuffer_t CAN_TxBuffer;
    CAN_RxBuffer_t CAN_RxBuffer;
} bsp_user_can_t;

typedef enum
{
    BSP_CAN_OK = 0,
    BSP_CAN_ERROR = 1,
    BSP_CAN_LINE_BUSY = 2,
    BSP_CAN_TIMEOUT = 3,
    BSP_CAN_INVALID_PARAM = 4,
    BSP_CAN_NOT_SUPPORTED = 5,
    BSP_CAN_UNKNOWN_ERROR = 6,
} bsp_can_state_e;

extern bsp_user_can_t bsp_user_can1;
extern bsp_user_can_t bsp_user_can2;
extern bsp_can_filt_tbl_t bsp_can1_filt_tbl;
extern bsp_can_filt_tbl_t bsp_can2_filt_tbl;
extern uint32_t can1_tx_ok_cnt;
extern uint32_t can2_tx_ok_cnt;

void bsp_can_init(bsp_user_can_t *canx, void (*pFunc)(CAN_RxBuffer_t *));
bsp_can_state_e bsp_can_std_transmit(can_channel_e can_periph, uint32_t id, uint8_t *data);

#ifdef __cplusplus
}
#endif
#endif
