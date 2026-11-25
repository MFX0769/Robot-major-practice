#include "prot_8irtraking.h"
#include "usart.h"
#include "usart_comm.h"
#include <stdio.h>

prot_8irtraking_data_t s_data;
static void compute_position(void);

/* 初始化：UART7 已由 main() 的 MX_UART7_Init 配好（AF7, PE7=RX/PE8=TX）。
   不要 DeInit —— 会破坏 AF7 引脚复用。 */
void prot_8irtraking_init(void)
{
    memset(&s_data, 0, sizeof(s_data));
    prot_8ir_start_rx();
}

/* 启动命令 $0,1,1# */
int prot_8irtraking_cmd_read(uint8_t analog_en, uint8_t digital_en)
{
    uint8_t buf[16];
    int n = snprintf((char *)buf, sizeof(buf), "$0,%d,%d#",
                     analog_en ? 1 : 0, digital_en ? 1 : 0);
    if (n > 0) {
        return (int)HAL_UART_Transmit(&LINE_HUART, buf, (uint16_t)n, 50);
    }
    return -1;
}

volatile uint8_t  ir_rx_byte;
volatile uint32_t ir_isr_bytes  = 0;
volatile uint32_t ir_poll_bytes = 0;
volatile uint32_t ir_diag_ticks = 0;
volatile uint32_t ir_last_isr   = 0;
volatile uint32_t ir_ore_count  = 0;
volatile uint32_t ir_pe7_changes = 0;
static char     ir_fbuf[96];
static uint16_t ir_flen = 0;

void prot_8ir_start_rx(void)
{
    if (LINE_HUART.RxState == HAL_UART_STATE_READY) {
        HAL_UART_Receive_IT(&LINE_HUART, (uint8_t *)&ir_rx_byte, 1);
    }
}

void prot_8ir_on_rx(void)
{
    uint8_t b = ir_rx_byte;
    ir_isr_bytes++;

    if (b == '$') {
        ir_flen = 0;
        ir_fbuf[ir_flen++] = (char)b;
    } else if (ir_flen > 0) {
        if (ir_flen < sizeof(ir_fbuf) - 1) ir_fbuf[ir_flen++] = (char)b;
        if (b == '#') {
            prot_8irtraking_on_receive((const uint8_t *)ir_fbuf, ir_flen);
            ir_flen = 0;
        }
    }
    HAL_UART_Receive_IT(&LINE_HUART, (uint8_t *)&ir_rx_byte, 1);
}

/* 纯轮询读 RDR（绕过 HAL 中断状态机） */
void prot_8ir_poll_rx(void)
{
    uint32_t guard = 0;
    ir_last_isr = LINE_HUART.Instance->ISR;

    if (ir_last_isr & (1U << 3)) {
        ir_ore_count++;
        volatile uint32_t sr = LINE_HUART.Instance->ISR; (void)sr;
        volatile uint32_t dr = LINE_HUART.Instance->RDR; (void)dr;
        LINE_HUART.Instance->ICR = (1U << 3);
    }

    while ((LINE_HUART.Instance->ISR & (1U << 5)) && guard < 128) {
        uint8_t b = (uint8_t)(LINE_HUART.Instance->RDR & 0xFFU);
        ir_poll_bytes++;
        guard++;

        if (b == '$') {
            ir_flen = 0;
            ir_fbuf[ir_flen++] = (char)b;
        } else if (ir_flen > 0) {
            if (ir_flen < sizeof(ir_fbuf) - 1) ir_fbuf[ir_flen++] = (char)b;
            if (b == '#') {
                prot_8irtraking_on_receive((const uint8_t *)ir_fbuf, ir_flen);
                ir_flen = 0;
            }
        }
    }
}

/* 主循环：中断负责接收；每 2s 补发一次启动命令 */
void prot_8ir_uart_diag(void)
{
    static uint32_t tick = 0;
    static uint8_t last_pe7 = 2;
    uint8_t now_pe7 = (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_7) == GPIO_PIN_SET) ? 1U : 0U;
    if (last_pe7 != 2 && now_pe7 != last_pe7) ir_pe7_changes++;
    last_pe7 = now_pe7;
    ir_diag_ticks++;
    tick++;
    if (tick % 1000 == 0) {
        prot_8irtraking_cmd_read(1, 1);
    }
}

static int parse_value(const char *p, const char *key, int *val)
{
    const char *k = strstr(p, key);
    if (!k) return -1;
    k += 3;
    *val = 0;
    while (*k >= '0' && *k <= '9') { *val = *val * 10 + (*k - '0'); k++; }
    return 0;
}

void prot_8irtraking_on_receive(const uint8_t *buf, uint16_t len)
{
    if (!buf || len < 8) return;
    s_data.rx_cnt++;

    const uint8_t *p = buf;
    const uint8_t *end = buf + len;

    while (p < end - 7) {
        if (*p != '$') { p++; continue; }
        const uint8_t *tail = (const uint8_t *)memchr(p + 1, '#', (size_t)(end - p - 1));
        if (!tail) break;

        char frame[128];
        uint16_t flen = (uint16_t)(tail - p + 1);
        if (flen >= sizeof(frame)) { p = tail + 1; continue; }
        memcpy(frame, p, flen);
        frame[flen] = '\0';

        if (frame[1] == 'D') {
            for (int i = 0; i < PROT_8IR_NUM_SENSORS; i++) {
                char key[4];
                snprintf(key, sizeof(key), "x%d", i + 1);
                int v = 0;
                if (parse_value(frame, key, &v) == 0)
                    s_data.digital[i] = (uint8_t)(v ? 1 : 0);
            }
        } else if (frame[1] == 'A') {
            for (int i = 0; i < PROT_8IR_NUM_SENSORS; i++) {
                char key[4];
                snprintf(key, sizeof(key), "x%d", i + 1);
                int v = 0;
                if (parse_value(frame, key, &v) == 0)
                    s_data.analog[i] = (uint16_t)v;
            }
            compute_position();   /* 模拟量驱动位置解算 (2026-11-23) */
        }
        p = tail + 1;
    }
}

/* 模拟量加权质心 + 丢线记忆 (2026-11-23)
   思路参照 25 电赛 line_follow_sim 的质心法, 升级为模拟量:
   - 自适应阈值 thr = min + RATIO*(max-min), 逐帧相对, 不依赖模块校准
   - 对比度门 MIN_CONTRAST: 全白/噪声帧(~20)不误触发 (模块边缘闪的问题)
   - 丢线/全压线时 position 保持记忆值, 由底盘按记忆方向给纠正量 */
#define LINE_BLACK_IS_LOW   1       /* 1: 黑线时模拟量变小(典型红外灰度) */
#define LINE_THR_RATIO      0.5f    /* 阈值位置 0~1 */
#define LINE_MIN_WEIGHT     0.08f   /* 单通道最低有效黑度 */
#define LINE_MIN_SUM        0.5f    /* 总黑度低于此=丢线 */
#define LINE_MIN_CONTRAST   150     /* 帧对比度低于此=无线 */

static void compute_position(void)
{
    uint16_t vmax = s_data.analog[0], vmin = s_data.analog[0];
    for (int i = 1; i < PROT_8IR_NUM_SENSORS; i++) {
        if (s_data.analog[i] > vmax) vmax = s_data.analog[i];
        if (s_data.analog[i] < vmin) vmin = s_data.analog[i];
    }
    s_data.contrast = (uint16_t)(vmax - vmin);

    if (s_data.contrast < LINE_MIN_CONTRAST) {   /* 无对比度: 丢线, 保持记忆 */
        s_data.state = LINE_STATE_LOST;
        s_data.on_cnt = 0;
        return;
    }

    float thr = vmin + LINE_THR_RATIO * (float)s_data.contrast;
    float wsum = 0.0f, csum = 0.0f;
    uint8_t on = 0;

    for (int i = 0; i < PROT_8IR_NUM_SENSORS; i++) {
        float blackness;   /* 0=白 1=全黑 */
#if LINE_BLACK_IS_LOW
        blackness = (thr - (float)s_data.analog[i]) / ((float)s_data.contrast + 0.001f);
#else
        blackness = ((float)s_data.analog[i] - thr) / ((float)s_data.contrast + 0.001f);
#endif
        if (blackness < LINE_MIN_WEIGHT) continue;
        if (blackness > 1.0f) blackness = 1.0f;
        wsum += blackness;
        csum += blackness * (i - 3.5f);   /* 探头等距, 中心对称 */
        on++;
    }
    s_data.on_cnt = on;

    if (on >= PROT_8IR_NUM_SENSORS) {
        s_data.state = LINE_STATE_ALLON;         /* 十字/全黑: 保持记忆位置 */
    } else if (wsum >= LINE_MIN_SUM) {
        s_data.state = LINE_STATE_OK;
        s_data.position = (int16_t)(csum / wsum * (100.0f / 3.5f));
    } else {
        s_data.state = LINE_STATE_LOST;          /* 丢线: 保持记忆位置 */
    }
}

const prot_8irtraking_data_t *prot_8irtraking_get_data(void)
{
    return &s_data;
}

/* UART7 单字节中断接收完成回调：重新挂接收并解析 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART7) {
        prot_8ir_on_rx();
    }
}

/* 溢出/帧/噪声错误后 HAL 会关掉接收中断并把状态置回 READY。
   不重新挂接收的话，巡线从此不再收数据直到重启。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != UART7) return;
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);
    if (huart->RxState == HAL_UART_STATE_READY) {
        (void)HAL_UART_Receive_IT(huart, (uint8_t *)&ir_rx_byte, 1);
    }
}
