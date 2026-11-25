/**
  * @file prot_dbus.c
  * @brief DBUS 遥控器接收 (UART5/PD2, 100000-8E1)
  * @note ISR 组帧入队; mode_switch_task 调用 dbus_process 解析。
  */
#include "prot_dbus.h"
#include "prot_dr16.h"
#include "prot_sbus.h"
#include "status_task.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include <string.h>

static QueueHandle_t s_queue;
static uint8_t s_raw[DBUS_FRAME_LEN];
static uint8_t s_raw_len;
static volatile uint32_t s_rx_count;
static volatile uint32_t s_last_tick;

void dbus_rx_isr(void)
{
    uint32_t isr = UART5->ISR;
    if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE)) {
        __HAL_UART_CLEAR_FLAG(&huart5, UART_CLEAR_OREF);
        __HAL_UART_CLEAR_FLAG(&huart5, UART_CLEAR_FEF);
        __HAL_UART_CLEAR_FLAG(&huart5, UART_CLEAR_NEF);
        s_raw_len = 0;
    }
    while (UART5->ISR & USART_ISR_RXNE_RXFNE) {
        uint8_t byte = (uint8_t)UART5->RDR;
        if (s_raw_len == 0U && byte != 0x00U) continue;
        if (s_raw_len < DBUS_FRAME_LEN) s_raw[s_raw_len++] = byte;
        if (s_raw_len == DBUS_FRAME_LEN) {
            dbus_frame_t frame;
            /* 原仓 18B 缓冲包含起始字节, prot_dr16 从 buff[0] 开始解通道。 */
            memcpy(frame.data, s_raw, DBUS_FRAME_LEN);
            if (s_queue != NULL) (void)xQueueSendFromISR(s_queue, &frame, NULL);
            s_raw_len = 0;
            s_rx_count++;
            s_last_tick = osKernelSysTick();
        }
    }
}

void dbus_init(void)
{
    s_queue = xQueueCreate(DBUS_QUEUE_LEN, sizeof(dbus_frame_t));
    s_raw_len = 0;
    s_rx_count = 0;
    s_last_tick = 0;
    __HAL_UART_DISABLE_IT(&huart5, UART_IT_RXNE);
    while (UART5->ISR & USART_ISR_RXNE_RXFNE) (void)UART5->RDR;
    __HAL_UART_CLEAR_FLAG(&huart5, UART_CLEAR_OREF);
    __HAL_UART_CLEAR_FLAG(&huart5, UART_CLEAR_FEF);
    __HAL_UART_CLEAR_FLAG(&huart5, UART_CLEAR_NEF);
    HAL_NVIC_SetPriority(UART5_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE);
}

void dbus_process(void)
{
    dbus_frame_t frame;
    if (s_queue == NULL || xQueueReceive(s_queue, &frame, 0) != pdTRUE) return;
    sbus_callback_handler(frame.data, DBUS_FRAME_LEN);
    wdg_user_set_bit(WDG_BIT_BSP_REMOTE);
}

uint8_t dbus_online(void)
{
    return s_last_tick != 0U && (uint32_t)(osKernelSysTick() - s_last_tick) < 100U;
}

uint32_t dbus_rx_count(void)
{
    return s_rx_count;
}
