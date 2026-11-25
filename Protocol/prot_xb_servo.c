/**
  * @file prot_xb_servo.c
  * @brief 星呗 SP 系列单线半双工总线舵机驱动
  * @note USART1(PA9) 115200-8N1。发送期间屏蔽 RX, TC 后恢复;
  *       ISR 只搬运字节, 帧解析与在线状态由机械臂任务周期维护。
 */
#include "prot_xb_servo.h"
#include "usart.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>

#define XB_RX_RING_SIZE 128U

static volatile uint8_t s_rx_ring[XB_RX_RING_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static volatile uint32_t s_rx_errors;
static char s_last_frame[40];
static uint32_t s_last_frame_tick;

xb_servo_dev_t xb_servo[XB_JOINT_NUM] = {
    {2, 0, XB_PULSE_ZERO, 0, 0, 0},   /* 肩实测硬件 ID=002 */
    {1, 0, XB_PULSE_ZERO, 0, 0, 0},   /* 肘实测硬件 ID=001 */
};

void xb_servo_rx_isr(void)
{
    uint32_t isr = USART1->ISR;
    if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE)) {
        s_rx_errors++;
        __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF);
        __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_FEF);
        __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_NEF);
    }
    while (USART1->ISR & USART_ISR_RXNE_RXFNE) {
        uint8_t byte = (uint8_t)USART1->RDR;
        uint16_t next = (uint16_t)((s_rx_head + 1U) % XB_RX_RING_SIZE);
        if (next != s_rx_tail) {
            s_rx_ring[s_rx_head] = byte;
            s_rx_head = next;
        } else {
            s_rx_errors++;
        }
    }
}

static void xb_servo_reset_rx(void)
{
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
    while (USART1->ISR & USART_ISR_RXNE_RXFNE) (void)USART1->RDR;
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF);
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_FEF);
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_NEF);
    s_rx_head = 0;
    s_rx_tail = 0;
}

void xb_servo_init(void)
{
    xb_servo_reset_rx();
    HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* 返回 0=TX 完整, 1=未初始化, 2=TX 超时。运动命令无应答, 只保证发送完整。 */
static uint8_t xb_servo_send_frame(const char *frame)
{
    size_t len = strlen(frame);
    uint32_t sent = 0;

    xb_servo_reset_rx();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t budget = SystemCoreClock / 500U;  /* 2ms */
    for (sent = 0; sent < len; sent++) {
        while (!(USART1->ISR & USART_ISR_TXE_TXFNF)) {
            if (DWT->CYCCNT > budget) {
                __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF);
                return 2U;
            }
        }
        USART1->TDR = (uint8_t)frame[sent];
    }
    while (!(USART1->ISR & USART_ISR_TC)) {
        if (DWT->CYCCNT > budget) {
            __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF);
            return 2U;
        }
    }
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF);
    return 0U;
}

uint8_t xb_servo_set_pos(uint8_t joint, uint16_t pulse, uint16_t time_ms)
{
    char frame[24];
    if (joint >= XB_JOINT_NUM) return 1U;
    if (pulse < XB_PULSE_MIN || pulse > XB_PULSE_MAX) return 1U;
    if (time_ms > XB_TIME_MAX_MS) time_ms = XB_TIME_MAX_MS;
    snprintf(frame, sizeof(frame), "#%03uP%04uT%04u!", xb_servo[joint].id, pulse, time_ms);
    return xb_servo_send_frame(frame);
}

uint8_t xb_servo_release(uint8_t joint)
{
    char frame[16];
    if (joint >= XB_JOINT_NUM) return 1U;
    snprintf(frame, sizeof(frame), "#%03uPULK!", xb_servo[joint].id);
    return xb_servo_send_frame(frame);
}

uint8_t xb_servo_lock(uint8_t joint)
{
    char frame[16];
    if (joint >= XB_JOINT_NUM) return 1U;
    snprintf(frame, sizeof(frame), "#%03uPULR!", xb_servo[joint].id);
    return xb_servo_send_frame(frame);
}

static void xb_servo_parse_frames(void)
{
    while (s_rx_tail != s_rx_head) {
        if (s_rx_ring[s_rx_tail] != '#') {
            s_rx_tail = (uint16_t)((s_rx_tail + 1U) % XB_RX_RING_SIZE);
            continue;
        }

        /* 台架实测响应以 ## 开头; 若第二个 '#' 尚未到达则保留, 下轮再解析。 */
        uint16_t start = s_rx_tail;
        uint16_t next = (uint16_t)((start + 1U) % XB_RX_RING_SIZE);
        if (next != s_rx_head && s_rx_ring[next] == '#') start = next;

        char frame[40];
        uint8_t len = 0;
        uint16_t idx = start;
        while (idx != s_rx_head && len < sizeof(frame) - 1U) {
            frame[len++] = (char)s_rx_ring[idx];
            idx = (uint16_t)((idx + 1U) % XB_RX_RING_SIZE);
            if (frame[len - 1U] == '!') break;
        }
        if (frame[len - 1U] != '!') {
            if (len == sizeof(frame) - 1U) s_rx_tail = (uint16_t)((s_rx_tail + 1U) % XB_RX_RING_SIZE);
            return;
        }
        s_rx_tail = idx;
        frame[len] = '\0';
        memcpy(s_last_frame, frame, len + 1U);
        s_last_frame_tick = osKernelSysTick();

        uint8_t id;
        uint16_t pulse;
        int16_t id16, temp, volt;
        if (len == 6U && frame[4] == 'P' &&
            sscanf(frame + 1, "%3hhuP", &id) == 1) {
            for (uint8_t j = 0; j < XB_JOINT_NUM; j++) {
                if (xb_servo[j].id == id) {
                    xb_servo[j].online = 1U;
                    xb_servo[j].last_ok_tick = osKernelSysTick();
                }
            }
            continue;
        }
        if (len == 10U && frame[4] == 'P' &&
            sscanf(frame + 1, "%3hhuP%4hu", &id, &pulse) == 2 && pulse <= XB_PULSE_MAX) {
            for (uint8_t j = 0; j < XB_JOINT_NUM; j++) {
                if (xb_servo[j].id == id) {
                    xb_servo[j].pulse = pulse;
                    xb_servo[j].online = 1U;
                    xb_servo[j].last_ok_tick = osKernelSysTick();
                }
            }
            continue;
        }
        if (len == 9U && frame[4] == 'T' &&
            sscanf(frame + 1, "%3hdT%2hdV%1hd", &id16, &temp, &volt) == 3) {
            for (uint8_t j = 0; j < XB_JOINT_NUM; j++) {
                if (xb_servo[j].id == (uint8_t)id16) {
                    xb_servo[j].temp_c = temp;
                    xb_servo[j].volt_v = volt;
                    xb_servo[j].last_ok_tick = osKernelSysTick();
                }
            }
        }
    }
}

uint8_t xb_servo_read_pos(uint8_t joint, uint16_t *pulse)
{
    char frame[16];
    if (joint >= XB_JOINT_NUM || pulse == NULL) return 1U;
    snprintf(frame, sizeof(frame), "#%03uPRAD!", xb_servo[joint].id);
    if (xb_servo_send_frame(frame) != 0U) return 2U;

    xb_servo[joint].online = 0U;
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    for (uint8_t i = 0; i < 10U; i++) {
        osDelay(1);
        xb_servo_parse_frames();
        if (xb_servo[joint].online) {
            *pulse = xb_servo[joint].pulse;
            __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
            return 0U;
        }
    }
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
    return 3U;
}

uint8_t xb_servo_read_id(uint8_t joint, uint8_t *id)
{
    char frame[16];
    if (joint >= XB_JOINT_NUM || id == NULL) return 1U;
    snprintf(frame, sizeof(frame), "#%03uPID!", xb_servo[joint].id);
    if (xb_servo_send_frame(frame) != 0U) return 2U;
    xb_servo[joint].online = 0U;
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    for (uint8_t i = 0; i < 10U; i++) {
        osDelay(1);
        xb_servo_parse_frames();
        if (xb_servo[joint].online) {
            *id = xb_servo[joint].id;
            __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
            return 0U;
        }
    }
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
    return 3U;
}

uint8_t xb_servo_read_id_raw(uint8_t servo_id, uint8_t *id)
{
    char frame[16];
    if (servo_id > 254U || id == NULL) return 1U;
    snprintf(frame, sizeof(frame), "#%03uPID!", servo_id);
    if (xb_servo_send_frame(frame) != 0U) return 2U;
    s_last_frame[0] = '\0';
    s_last_frame_tick = 0;
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    for (uint8_t i = 0; i < 10U; i++) {
        osDelay(1);
        xb_servo_parse_frames();
        if (s_last_frame[0] != '\0') {
            uint8_t replied_id;
            if (sscanf(s_last_frame + 1, "%3hhuP", &replied_id) == 1) {
                *id = replied_id;
                __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
                return 0U;
            }
        }
    }
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
    return 3U;
}

uint8_t xb_servo_write_id_raw(uint8_t old_id, uint8_t new_id)
{
    char frame[20];
    uint32_t start;
    if (old_id > 254U || new_id > 254U || old_id == new_id) return 1U;
    snprintf(frame, sizeof(frame), "#%03uPID%03u!", old_id, new_id);
    if (xb_servo_send_frame(frame) != 0U) return 2U;
    s_last_frame[0] = '\0';
    s_last_frame_tick = 0;
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    start = osKernelSysTick();
    while ((uint32_t)(osKernelSysTick() - start) < 20U) {
        osDelay(1);
        xb_servo_parse_frames();
        if (s_last_frame[0] != '\0') {
            uint8_t replied_id;
            if (sscanf(s_last_frame + 1, "%3hhuP", &replied_id) == 1 && replied_id == new_id) {
                /* 只确认硬件 ID 已改, 不改关节映射; 必须显式 servo bind。 */
                __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
                return 0U;
            }
        }
    }
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
    return 3U;
}

uint8_t xb_servo_ping_raw(uint8_t id)
{
    char frame[16];
    uint32_t start;
    if (id > 254U) return 1U;
    snprintf(frame, sizeof(frame), "#%03uPVER!", id);
    if (xb_servo_send_frame(frame) != 0U) return 2U;
    s_last_frame[0] = '\0';
    s_last_frame_tick = 0;
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    start = osKernelSysTick();
    while ((uint32_t)(osKernelSysTick() - start) < 20U) {
        osDelay(1);
        xb_servo_parse_frames();
        if (s_last_frame[0] != '\0' && (uint32_t)(osKernelSysTick() - s_last_frame_tick) < 20U) {
            __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
            return 0U;
        }
    }
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
    return 3U;
}

const char *xb_servo_last_frame(void)
{
    return s_last_frame;
}

void xb_servo_poll(void)
{
    xb_servo_parse_frames();
    uint32_t now = osKernelSysTick();
    for (uint8_t j = 0; j < XB_JOINT_NUM; j++) {
        if (xb_servo[j].online && (uint32_t)(now - xb_servo[j].last_ok_tick) > XB_ONLINE_TIMEOUT) {
            xb_servo[j].online = 0U;
        }
    }
}

uint16_t xb_servo_get_pulse(uint8_t joint)
{
    return joint < XB_JOINT_NUM ? xb_servo[joint].pulse : 0U;
}

uint8_t xb_servo_online(uint8_t joint)
{
    return joint < XB_JOINT_NUM ? xb_servo[joint].online : 0U;
}

uint8_t xb_servo_bind(uint8_t joint, uint8_t id)
{
    if (joint >= XB_JOINT_NUM || id > 254U) return 1U;
    for (uint8_t j = 0; j < XB_JOINT_NUM; j++) {
        if (j != joint && xb_servo[j].id == id) return 2U;
    }
    xb_servo[joint].id = id;
    xb_servo[joint].online = 0U;
    return 0U;
}

uint32_t xb_servo_rx_errors(void)
{
    return s_rx_errors;
}
