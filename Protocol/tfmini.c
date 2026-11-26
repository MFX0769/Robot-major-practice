/**
  * @file tfmini.c
  * @brief 北醒 TF-mini Plus 前向测距驱动（UART9_RX = PD14）
  * @note 这块板的 USART2/3 只有 RS485 接口、UART8 给了视觉，空闲的 TTL 接收只剩 UART9。
  *       帧格式和校验照北醒官方例程（TFmini/TFmini-STM32 的 USART1_RX_Proc）；这里不用
  *       IDLE+DMA，改成逐字节状态机，帧错位后下一帧自动对齐。
  */
#include "tfmini.h"
#include "main.h"
#include "usart.h"
#include "cmsis_os.h"
#include <string.h>

tfmini_data_t tfmini[TFMINI_NUM];

static uint8_t s_raw[TFMINI_FRAME_LEN];
static uint8_t s_len;
static uint8_t s_frame_ok;      /* 最近一帧校验通过且强度可信 */

/* 中断里逐字节喂进来 */
static void tfmini_feed(uint8_t byte)
{
    uint8_t i, sum = 0U;
    uint16_t dist, strength;

    if (s_len == 0U) {
        if (byte == TFMINI_HDR) s_raw[s_len++] = byte;
        return;
    }
    if (s_len == 1U && byte != TFMINI_HDR) {
        s_len = 0U;                     /* 0x59 后面不是 0x59：不是帧头，重来 */
        return;
    }
    s_raw[s_len++] = byte;
    if (s_len < TFMINI_FRAME_LEN) return;

    s_len = 0U;
    for (i = 0U; i < (uint8_t)(TFMINI_FRAME_LEN - 1U); i++) sum = (uint8_t)(sum + s_raw[i]);
    if (sum != s_raw[TFMINI_FRAME_LEN - 1U]) {
        tfmini[0].crc_errors++;
        s_frame_ok = 0U;
        /* 校验失败时滑动到窗口里最后一对帧头。连续两个 0x59（噪声尾 + 下一帧头）
           不能让下一帧从中间开始，否则会连丢两帧。 */
        for (i = (uint8_t)(TFMINI_FRAME_LEN - 1U); i > 0U; i--) {
            if (s_raw[i - 1U] == TFMINI_HDR && s_raw[i] == TFMINI_HDR) {
                uint8_t keep = (uint8_t)(TFMINI_FRAME_LEN - (i - 1U));
                if (i == 1U) keep = 1U;   /* 窗口开头那对也算：只留第一个头等下一个，别把 9B 全留下 */
                memmove(s_raw, s_raw + (i - 1U), keep);
                s_len = keep;
                return;
            }
        }
        s_len = 0U;
        return;
    }

    dist = (uint16_t)((uint16_t)s_raw[2] | ((uint16_t)s_raw[3] << 8));
    strength = (uint16_t)((uint16_t)s_raw[4] | ((uint16_t)s_raw[5] << 8));
    tfmini[0].dist_raw = dist;
    tfmini[0].strength = strength;
    tfmini[0].temp_c = (int16_t)(((int16_t)((uint16_t)s_raw[6] | ((uint16_t)s_raw[7] << 8))) / 8 - 256);
    tfmini[0].rx_frames++;
    tfmini[0].last_tick = osKernelSysTick();
    s_frame_ok = (uint8_t)((strength >= TFMINI_STRENGTH_MIN && dist != 0xFFFFU && dist != 0U) ? 1U : 0U);
}

void tfmini_rx_isr(void)
{
    uint32_t isr = UART9->ISR;

    if ((isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE)) != 0U) {
        __HAL_UART_CLEAR_FLAG(&huart9, UART_CLEAR_OREF);
        __HAL_UART_CLEAR_FLAG(&huart9, UART_CLEAR_FEF);
        __HAL_UART_CLEAR_FLAG(&huart9, UART_CLEAR_NEF);
        s_len = 0U;                     /* 出错：丢掉半帧，下一帧头重新对齐 */
    }
    while ((UART9->ISR & USART_ISR_RXNE_RXFNE) != 0U) {
        tfmini[0].rx_bytes++;
        tfmini_feed((uint8_t)UART9->RDR);
    }
}

void tfmini_init(void)
{
    GPIO_InitTypeDef g = {0};

    memset(tfmini, 0, sizeof(tfmini));
    s_len = 0U;
    s_frame_ok = 0U;

    /* PD14 -> UART9_RX。上拉：没插模块时引脚不悬空 */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    g.Pin = GPIO_PIN_14;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF11_UART9;
    HAL_GPIO_Init(GPIOD, &g);

    __HAL_RCC_UART9_CLK_ENABLE();
    huart9.Instance = UART9;
    huart9.Init.BaudRate = 115200;
    huart9.Init.WordLength = UART_WORDLENGTH_8B;
    huart9.Init.StopBits = UART_STOPBITS_1;
    huart9.Init.Parity = UART_PARITY_NONE;
    huart9.Init.Mode = UART_MODE_RX;
    huart9.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart9.Init.OverSampling = UART_OVERSAMPLING_16;
    huart9.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart9.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart9.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    (void)HAL_UART_Init(&huart9);

    HAL_NVIC_SetPriority(UART9_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(UART9_IRQn);
    __HAL_UART_ENABLE_IT(&huart9, UART_IT_RXNE);
}

uint8_t tfmini_online(uint8_t id)
{
    if (id >= TFMINI_NUM) return 0U;
    return (tfmini[id].last_tick != 0U &&
            (uint32_t)(osKernelSysTick() - tfmini[id].last_tick) <= TFMINI_MAX_AGE_MS) ? 1U : 0U;
}

uint16_t tfmini_distance(uint8_t id)
{
    return (id < TFMINI_NUM) ? tfmini[id].dist_raw : 0U;
}

uint8_t tfmini_frame_valid(uint8_t id)
{
    return (id < TFMINI_NUM) ? s_frame_ok : 0U;
}
