/**
  ******************************************************************************
  * @file    bsp_can.c
  * @brief   CAN1：4×2006；CAN2：6020。所有过滤规则进入 FIFO0。
  ******************************************************************************
  */
#include "bsp_can.h"

static const FDCAN_TxHeaderTypeDef default_tx_header = {
    .IdType = FDCAN_STANDARD_ID,
    .TxFrameType = FDCAN_DATA_FRAME,
    .DataLength = FDCAN_DLC_BYTES_8,
    .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
    .BitRateSwitch = FDCAN_BRS_OFF,
    .FDFormat = FDCAN_CLASSIC_CAN,
    .TxEventFifoControl = FDCAN_STORE_TX_EVENTS,
    .MessageMarker = 0,
};

uint32_t can1_tx_ok_cnt;
uint32_t can2_tx_ok_cnt;
static void (*pCAN1_RxCpltCallback)(CAN_RxBuffer_t *);
static void (*pCAN2_RxCpltCallback)(CAN_RxBuffer_t *);

/* [0] = CAN1, [1] = CAN2 */
typedef struct
{
    uint32_t tx_ok, rx, bus_off, recovered;
} bsp_can_cnt_t;
static bsp_can_cnt_t s_cnt[2];

bsp_user_can_t bsp_user_can1 = {
    .hfdcan = &hfdcan1,
    .CAN_TxBuffer = {0},
    .CAN_RxBuffer = {0},
};
bsp_user_can_t bsp_user_can2 = {
    .hfdcan = &hfdcan2,
    .CAN_TxBuffer = {0},
    .CAN_RxBuffer = {0},
};
bsp_can_filt_tbl_t bsp_can1_filt_tbl;
bsp_can_filt_tbl_t bsp_can2_filt_tbl;

static void bsp_can_filter_init(bsp_user_can_t *canx)
{
    FDCAN_FilterTypeDef filter = {0};
    const bsp_can_filt_tbl_t *table;

    if (canx->hfdcan == &hfdcan1) table = &bsp_can1_filt_tbl;
    else if (canx->hfdcan == &hfdcan2) table = &bsp_can2_filt_tbl;
    else return;
    if (table->count > 10U) Error_Handler();

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    for (uint8_t i = 0; i < table->count; i++)
    {
        filter.FilterIndex = i;
        switch (table->data[i * 3U])
        {
            case FLT_RANGE:  filter.FilterType = FDCAN_FILTER_RANGE; break;
            case FLT_DUAL:   filter.FilterType = FDCAN_FILTER_DUAL; break;
            case FLT_MASK:   filter.FilterType = FDCAN_FILTER_MASK; break;
            default: Error_Handler(); return;
        }
        filter.FilterID1 = table->data[i * 3U + 1U] & 0x7FFU;
        filter.FilterID2 = table->data[i * 3U + 2U] & 0x7FFU;
        if (HAL_FDCAN_ConfigFilter(canx->hfdcan, &filter) != HAL_OK)
        {
            Error_Handler();
        }
    }

    HAL_FDCAN_ConfigGlobalFilter(canx->hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    /* BUS_OFF 中断打开：掉电上电顺序、接线错误都会进 bus-off，进去了就自动恢复 */
    if (HAL_FDCAN_ActivateNotification(canx->hfdcan,
                                       FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF, 0) != HAL_OK)
    {
        Error_Handler();
    }
}

void bsp_can_init(bsp_user_can_t *canx, void (*pFunc)(CAN_RxBuffer_t *))
{
    if (canx == NULL || pFunc == NULL) Error_Handler();
    bsp_can_filter_init(canx);
    if (canx->hfdcan == &hfdcan1) pCAN1_RxCpltCallback = pFunc;
    else if (canx->hfdcan == &hfdcan2) pCAN2_RxCpltCallback = pFunc;
    else Error_Handler();
    if (HAL_FDCAN_Start(canx->hfdcan) != HAL_OK) Error_Handler();
}

bsp_can_state_e bsp_can_std_transmit(can_channel_e can_periph, uint32_t id, uint8_t *data)
{
    FDCAN_HandleTypeDef *hfdcan;
    FDCAN_TxHeaderTypeDef header = default_tx_header;

    if (data == NULL || id > 0x7FFU) return BSP_CAN_INVALID_PARAM;
    if (can_periph == CAN_CHANNEL_1) hfdcan = &hfdcan1;
    else if (can_periph == CAN_CHANNEL_2) hfdcan = &hfdcan2;
    else return BSP_CAN_INVALID_PARAM;

    header.Identifier = id;
    switch (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &header, data))
    {
        case HAL_OK:
            if (can_periph == CAN_CHANNEL_1) { can1_tx_ok_cnt++; s_cnt[0].tx_ok++; }
            else { can2_tx_ok_cnt++; s_cnt[1].tx_ok++; }
            return BSP_CAN_OK;
        case HAL_BUSY:
            return BSP_CAN_LINE_BUSY;
        default:
            return BSP_CAN_ERROR;
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    bsp_user_can_t *canx;
    void (*callback)(CAN_RxBuffer_t *);

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) return;
    if (hfdcan->Instance == FDCAN1)
    {
        canx = &bsp_user_can1;
        callback = pCAN1_RxCpltCallback;
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        canx = &bsp_user_can2;
        callback = pCAN2_RxCpltCallback;
    }
    else return;

    if (callback != NULL &&
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &canx->CAN_RxBuffer.rxHeader,
                               canx->CAN_RxBuffer.rx_data) == HAL_OK)
    {
        s_cnt[(hfdcan->Instance == FDCAN1) ? 0U : 1U].rx++;
        callback(&canx->CAN_RxBuffer);
    }
}

/* 进 bus-off 后在总线上不再收发。置 INIT 再清掉，节点在 128 个 11 位隐性位后重新同步；
   过滤表和中断使能都保留。不恢复的话只能复位整块板。 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    uint8_t idx = (hfdcan->Instance == FDCAN1) ? 0U : 1U;

    if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) == 0U) return;
    s_cnt[idx].bus_off++;
    if (HAL_FDCAN_Stop(hfdcan) == HAL_OK && HAL_FDCAN_Start(hfdcan) == HAL_OK)
    {
        s_cnt[idx].recovered++;
    }
}

void bsp_can_stat_get(can_channel_e ch, bsp_can_stat_t *out)
{
    FDCAN_HandleTypeDef *hfdcan = (ch == CAN_CHANNEL_1) ? &hfdcan1 : &hfdcan2;
    FDCAN_ErrorCountersTypeDef ec = {0};
    uint32_t psr;
    uint8_t idx = (ch == CAN_CHANNEL_1) ? 0U : 1U;

    if (out == NULL) return;
    out->tx_ok = s_cnt[idx].tx_ok;
    out->rx = s_cnt[idx].rx;
    out->bus_off = s_cnt[idx].bus_off;
    out->recovered = s_cnt[idx].recovered;
    psr = hfdcan->Instance->PSR;
    out->bo = ((psr & FDCAN_PSR_BO) != 0U) ? 1U : 0U;
    out->lec = (uint8_t)((psr & FDCAN_PSR_LEC) >> FDCAN_PSR_LEC_Pos);
    out->tec = 0U;
    out->rec = 0U;
    if (HAL_FDCAN_GetErrorCounters(hfdcan, &ec) == HAL_OK)
    {
        out->tec = (uint8_t)ec.TxErrorCnt;
        out->rec = (uint8_t)ec.RxErrorCnt;
    }
}

void bsp_can_recover(can_channel_e ch)
{
    FDCAN_HandleTypeDef *hfdcan = (ch == CAN_CHANNEL_1) ? &hfdcan1 : &hfdcan2;

    if (HAL_FDCAN_Stop(hfdcan) == HAL_OK)
    {
        (void)HAL_FDCAN_Start(hfdcan);
    }
}
