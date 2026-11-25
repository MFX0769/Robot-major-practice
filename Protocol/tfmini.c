/**
  * @file tfmini.c
  * @brief 北醒 TF-mini Plus 四路 UART 测距驱动
  * @note ISR 只做帧解析; 统一 poll 维护数据年龄。传感器失效时返回 offline,
  *       由避障逻辑决定是否保守停车, 不把 0 距离当成真实近障。
 */
#include "tfmini.h"
#include "cmsis_os.h"
#include <string.h>

tfmini_data_t tfmini[TFMINI_NUM];

static uint8_t s_raw[TFMINI_NUM][TFMINI_FRAME_LEN];
static uint8_t s_len[TFMINI_NUM];

void tfmini_rx_isr(uint8_t id)
{
    USART_TypeDef *uart;
    if (id >= TFMINI_NUM) return;
    switch (id) {
        case TFMINI_FRONT: uart = USART2; break;
        case TFMINI_LEFT:  uart = USART3; break;
        case TFMINI_RIGHT: uart = UART8;  break;
        case TFMINI_REAR:  uart = USART9; break;
        default: return;
    }

    while (uart->ISR & USART_ISR_RXNE_RXFNE) {
        uint8_t byte = (uint8_t)uart->RDR;
        if (s_len[id] == 0U && byte != 0x59U) continue;
        if (s_len[id] == 1U && byte != 0x59U) {
            s_len[id] = (byte == 0x59U) ? 1U : 0U;
            continue;
        }
        s_raw[id][s_len[id]++] = byte;
        if (s_len[id] < TFMINI_FRAME_LEN) continue;

        uint8_t sum = 0;
        for (uint8_t i = 0; i < TFMINI_FRAME_LEN - 1U; i++) sum += s_raw[id][i];
        if (sum == s_raw[id][TFMINI_FRAME_LEN - 1U]) {
            tfmini[id].distance_mm = (uint16_t)s_raw[id][2] | ((uint16_t)s_raw[id][3] << 8);
            tfmini[id].strength = (uint16_t)s_raw[id][4] | ((uint16_t)s_raw[id][5] << 8);
            int16_t raw_temp = (int16_t)s_raw[id][6] | ((int16_t)s_raw[id][7] << 8);
            tfmini[id].temp_c = raw_temp / 8 - 256;
            tfmini[id].last_tick = osKernelSysTick();
            tfmini[id].rx_frames++;
        } else {
            tfmini[id].crc_errors++;
        }
        s_len[id] = 0U;
    }
}

void tfmini_init(void)
{
    memset(tfmini, 0, sizeof(tfmini));
    memset(s_len, 0, sizeof(s_len));
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
    __HAL_UART_ENABLE_IT(&huart8, UART_IT_RXNE);
    __HAL_UART_ENABLE_IT(&huart9, UART_IT_RXNE);
}

void tfmini_poll(void)
{
    for (uint8_t i = 0; i < TFMINI_NUM; i++) {
        if (tfmini[i].last_tick != 0U && (uint32_t)(osKernelSysTick() - tfmini[i].last_tick) > TFMINI_MAX_AGE_MS) {
            tfmini[i].distance_mm = 0U;
        }
    }
}

uint8_t tfmini_online(uint8_t id)
{
    return id < TFMINI_NUM && tfmini[id].last_tick != 0U &&
           (uint32_t)(osKernelSysTick() - tfmini[id].last_tick) <= TFMINI_MAX_AGE_MS;
}

uint16_t tfmini_distance(uint8_t id)
{
    return tfmini_online(id) ? tfmini[id].distance_mm : 0U;
}
