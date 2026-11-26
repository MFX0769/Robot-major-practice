/* 队列、串口接收模型、can 帧记录。控制台、DBUS、底盘测试用。 */
#include <stdio.h>
#include <string.h>
#include "sim.h"
#include "main.h"
#include "queue.h"
#include "prot_can_send_interface.h"

#define UART_RX_CAP 2048U

typedef struct {
    uint8_t  buf[UART_RX_CAP];
    uint32_t head, tail;
    uint32_t err;                      /* ORE/FE/NE */
    uint32_t idle;                     /* 1 = 上次出队之后总线空闲过 */
    uint32_t icr;                      /* 上一次写进 ICR 的值 */
} uart_t;

static uart_t s_uart[3];
UART_HandleTypeDef huart5;             /* 测试里把 .uart 填成 5 */
UART_HandleTypeDef huart10;            /* .uart = 10 */
UART_HandleTypeDef huart9;             /* 测距模块，.uart = 9 */

static uart_t *uart_of(int uart)
{
    return (uart == 5) ? &s_uart[0] : (uart == 9) ? &s_uart[2] : &s_uart[1];
}

HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *h)
{
    (void)h;
    return HAL_OK;                     /* 波特率、引脚都是真的才有意义，这里只要能编过 */
}

static uint32_t uart_count(uart_t *u)
{
    return (u->head >= u->tail) ? (u->head - u->tail) : (UART_RX_CAP - u->tail + u->head);
}

/* 每次访问寄存器之前调用：处理上一次写进 ICR 的清除请求 */
static void uart_sweep(uart_t *u)
{
    if (u->icr != 0U) {
        if (u->icr & USART_ICR_IDLECF) u->idle = 0U;
        if (u->icr & USART_ICR_ORECF) u->err &= ~USART_ISR_ORE;
        if (u->icr & USART_ICR_FECF) u->err &= ~USART_ISR_FE;
        if (u->icr & USART_ICR_NECF) u->err &= ~USART_ISR_NE;
        u->icr = 0U;
    }
}

uint32_t sim_uart_isr(int uart)
{
    uart_t *u = uart_of(uart);
    uart_sweep(u);
    return u->err | (u->idle ? USART_ISR_IDLE : 0U) |
           ((uart_count(u) != 0U) ? USART_ISR_RXNE_RXFNE : 0U);
}

uint8_t sim_uart_rdr(int uart)
{
    uart_t *u = uart_of(uart);
    uint8_t b;
    uart_sweep(u);
    if (uart_count(u) == 0U) return 0U;
    b = u->buf[u->tail];
    u->tail = (u->tail + 1U) % UART_RX_CAP;
    return b;
}

volatile uint32_t *sim_uart_icr(int uart)
{
    uart_t *u = uart_of(uart);
    uart_sweep(u);
    return &u->icr;
}

void sim_uart_rx_push(int uart, uint8_t byte)
{
    uart_t *u = uart_of(uart);
    uint32_t next = (u->head + 1U) % UART_RX_CAP;
    if (next != u->tail) {
        u->buf[u->head] = byte;
        u->head = next;
    } else {
        u->err |= USART_ISR_ORE;       /* 没人取，溢出 */
    }
}

void sim_uart_rx_idle(int uart)
{
    uart_of(uart)->idle = 1U;
}

void sim_uart_rx_error(int uart, uint32_t flags)
{
    uart_of(uart)->err |= flags;
}

void sim_uart_clear_flag(int uart, uint32_t flags)
{
    uart_of(uart)->icr |= flags;       /* 和硬件一样：写 ICR，下一次读寄存器时生效 */
}

void HAL_NVIC_SetPriority(int irq, uint32_t preempt, uint32_t sub) { (void)irq; (void)preempt; (void)sub; }
void HAL_NVIC_EnableIRQ(int irq) { (void)irq; }

/* ---------------- 简单队列 ---------------- */
typedef struct {
    uint32_t len, size, count;
    uint8_t  *store;
} queue_sim_t;
static queue_sim_t s_queues[8];
static uint8_t s_qstore[8][1024];
static uint32_t s_qn;

QueueHandle_t xQueueCreate(uint32_t len, uint32_t item_size)
{
    queue_sim_t *q;
    if (s_qn >= 8U || len * item_size > sizeof(s_qstore[0])) return NULL;
    q = &s_queues[s_qn++];
    q->len = len;
    q->size = item_size;
    q->count = 0U;
    q->store = s_qstore[s_qn - 1U];
    return q;
}

BaseType_t xQueueSend(QueueHandle_t q, const void *item, uint32_t wait)
{
    (void)wait;
    return xQueueSendFromISR(q, item, NULL);
}

BaseType_t xQueueSendFromISR(QueueHandle_t q, const void *item, void *woken)
{
    queue_sim_t *s = (queue_sim_t *)q;
    (void)woken;
    if (s == NULL || s->count >= s->len) return pdFALSE;
    memcpy(s->store + s->count * s->size, item, s->size);
    s->count++;
    return pdTRUE;
}

BaseType_t xQueueReceive(QueueHandle_t q, void *buf, uint32_t wait)
{
    queue_sim_t *s = (queue_sim_t *)q;
    uint32_t i;
    (void)wait;
    if (s == NULL || s->count == 0U) return pdFALSE;
    memcpy(buf, s->store, s->size);
    for (i = 1U; i < s->count; i++) {
        memcpy(s->store + (i - 1U) * s->size, s->store + i * s->size, s->size);
    }
    s->count--;
    return pdTRUE;
}

/* ---------------- can 帧记录 ---------------- */
static void can_log(uint32_t ch, uint32_t id, const uint8_t *data, uint8_t dlc)
{
    g_canraw_ch = ch;
    g_canraw_id = id;
    g_canraw_n = dlc;
    memcpy(g_canraw_d, data, 8U);
    g_canraw_cnt++;
}

void Prot_CAN1_Send_Frame(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    can_log(1U, id, data, dlc);
}

/* 6020 在 CAN2：控制帧 0x1FF，ID4 占第 6、7 字节。模拟电机吃的就是这两字节。 */
void Prot_CAN2_Send_Frame(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    can_log(2U, id, data, dlc);
    if (id == 0x1FFU) {
        g_yaw_tx = (int16_t)(((uint16_t)data[6] << 8) | data[7]);
        g_yaw_frames++;
    }
}
