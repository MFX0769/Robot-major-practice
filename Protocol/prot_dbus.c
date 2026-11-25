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
static volatile uint32_t s_rx_bytes;
static volatile uint32_t s_last_tick;

static void dbus_push_frame(void)
{
    dbus_frame_t frame;

    if (s_queue != NULL) {
        memcpy(frame.data, s_raw, DBUS_FRAME_LEN);
        (void)xQueueSendFromISR(s_queue, &frame, NULL);
    }
    s_rx_count++;
    s_last_tick = osKernelSysTick();
}

/* 帧边界靠 IDLE：DBUS 每帧 18 字节，发完总线空闲十几个毫秒。
   达妙官方例程也是 ReceiveToIdle。不能拿某个字节的内容当帧头：
   buff[0] 是右摇杆横向通道的低字节，摇杆一动它就不是 0 了。 */
void dbus_rx_isr(void)
{
    uint32_t isr = UART5->ISR;

    if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE)) {
        __HAL_UART_CLEAR_FLAG(&huart5, UART_CLEAR_OREF);
        __HAL_UART_CLEAR_FLAG(&huart5, UART_CLEAR_FEF);
        __HAL_UART_CLEAR_FLAG(&huart5, UART_CLEAR_NEF);
        s_raw_len = DBUS_FRAME_LEN + 1U;      /* 出错：丢到下一个 IDLE 重新对齐 */
    }

    while (UART5->ISR & USART_ISR_RXNE_RXFNE) {
        uint8_t byte = (uint8_t)UART5->RDR;
        s_rx_bytes++;
        if (s_raw_len < DBUS_FRAME_LEN) {
            s_raw[s_raw_len++] = byte;
        } else {
            s_raw_len = DBUS_FRAME_LEN + 1U;  /* 溢出标记 */
        }
    }

    if ((UART5->ISR & USART_ISR_IDLE) != 0U) {
        UART5->ICR = USART_ICR_IDLECF;
        if (s_raw_len == DBUS_FRAME_LEN) dbus_push_frame();
        s_raw_len = 0U;
    }
}

void dbus_init(void)
{
    s_queue = xQueueCreate(DBUS_QUEUE_LEN, sizeof(dbus_frame_t));
    s_raw_len = 0;
    s_rx_count = 0;
    s_rx_bytes = 0;
    s_last_tick = 0;
    __HAL_UART_DISABLE_IT(&huart5, UART_IT_RXNE);
    while (UART5->ISR & USART_ISR_RXNE_RXFNE) (void)UART5->RDR;
    __HAL_UART_CLEAR_FLAG(&huart5, UART_CLEAR_OREF);
    __HAL_UART_CLEAR_FLAG(&huart5, UART_CLEAR_FEF);
    __HAL_UART_CLEAR_FLAG(&huart5, UART_CLEAR_NEF);
    UART5->ICR = USART_ICR_IDLECF;
    HAL_NVIC_SetPriority(UART5_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE | UART_IT_IDLE);
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

uint32_t dbus_rx_bytes(void)
{
    return s_rx_bytes;
}
