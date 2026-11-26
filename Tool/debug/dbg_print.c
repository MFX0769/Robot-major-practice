/**
  * @file    dbg_print.c
  * @brief   非阻塞调试打印
  * @note    格式化在调用者栈上完成，入队后由低优先级 dbg_task 发送。
  *          调度器未启动时直接轮询发送（仅启动横幅）。
  */
#include "dbg_print.h"
#include "usart.h"
#include "usbd_cdc_if.h"
#include "usb_comm.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define DBG_MSG_MAX   192U
#define DBG_QUEUE_LEN 8U

typedef struct {
    uint16_t len;
    char     data[DBG_MSG_MAX];
} dbg_msg_t;

static QueueHandle_t s_queue;
static volatile uint32_t s_enqueued, s_dropped, s_truncated, s_uart_tx, s_uart_timeout;

void dbg_stats_get(uint32_t *enq, uint32_t *drop, uint32_t *trunc, uint32_t *tx, uint32_t *timeout)
{
    if (enq != NULL) *enq = s_enqueued;
    if (drop != NULL) *drop = s_dropped;
    if (trunc != NULL) *trunc = s_truncated;
    if (tx != NULL) *tx = s_uart_tx;
    if (timeout != NULL) *timeout = s_uart_timeout;
}

void dbg_init(void)
{
    if (s_queue == NULL) {
        s_queue = xQueueCreate(DBG_QUEUE_LEN, sizeof(dbg_msg_t));
    }
    dbg_print("\r\n=== System Boot ===\r\n");
}

int dbg_print(const char *fmt, ...)
{
    char buf[DBG_MSG_MAX];
    va_list args;
    int len;
    uint16_t n;

    if (fmt == NULL) return 0;
    va_start(args, fmt);
    len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (len <= 0) return 0;
    n = (len >= (int)sizeof(buf)) ? (uint16_t)(sizeof(buf) - 1U) : (uint16_t)len;
    if ((uint16_t)len != n) s_truncated++;

    if (s_queue != NULL && xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        dbg_msg_t msg;
        msg.len = n;
        memcpy(msg.data, buf, n);
        if (xQueueSend(s_queue, &msg, 0) == pdPASS) s_enqueued++;
        else s_dropped++;                       /* 诊断可见：日志满了是丢日志，不是控制任务卡住 */
        usb_tx_send((const uint8_t *)buf, n);    /* 第二出口，尽力而为 */
    } else {
        if (HAL_UART_Transmit(&huart10, (uint8_t *)buf, n, 20U) != HAL_OK) s_uart_timeout++;
        else s_uart_tx++;
    }
    return (int)n;
}

void dbg_task(void const *argu)
{
    dbg_msg_t msg;
    (void)argu;
    if (s_queue == NULL) {
        s_queue = xQueueCreate(DBG_QUEUE_LEN, sizeof(dbg_msg_t));
    }
    for (;;) {
        if (s_queue != NULL &&
            xQueueReceive(s_queue, &msg, portMAX_DELAY) == pdPASS) {
            if (HAL_UART_Transmit(&huart10, (uint8_t *)msg.data, msg.len, 100U) == HAL_OK) s_uart_tx++;
            else s_uart_timeout++;
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void dbg_log(const char *tag, const char *msg)
{
    dbg_print("[%s] %s\r\n", tag, msg);
}

void dbg_hex(const char *tag, const uint8_t *data, uint16_t len)
{
    uint16_t i;
    dbg_print("[%s] HEX(%u): ", tag, (unsigned)len);
    for (i = 0; i < len && i < 32U; i++) dbg_print("%02X ", data[i]);
    dbg_print("\r\n");
}
