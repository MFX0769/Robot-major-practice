#include "key_task.h"
#include "adc.h"
#include "cmsis_os.h"
#include "bsp_buzzer.h"
#include "ws2812.h"
#include "arm_task.h"
#include "prot_rm_motor.h"
#include "dbg_print.h"

float vbus = 0;
volatile uint16_t adc_val[2];

static uint8_t  s_last_state = 0xFFU;
static uint32_t s_beep_off_tick;
static uint32_t s_can_tick;

/* 状态灯语：IDLE 绿 / READY 蓝 / RECORD 红 / SAVED 黄 / PLAYBACK 青 / ABORT 紫闪 */
static void led_show(uint8_t st)
{
    uint32_t t = HAL_GetTick();

    switch (st) {
        case ARM_TEACH_IDLE:     WS2812_Ctrl(0, 8, 0);    break;
        case ARM_TEACH_READY:    WS2812_Ctrl(0, 0, 12);   break;
        case ARM_TEACH_RECORD:   WS2812_Ctrl(12, 0, 0);   break;
        case ARM_TEACH_SAVED:    WS2812_Ctrl(12, 10, 0);  break;
        case ARM_TEACH_PLAYBACK: WS2812_Ctrl(0, 10, 10);  break;
        case ARM_TEACH_ABORT:
            if (((t / 300U) & 1U) != 0U) WS2812_Ctrl(10, 0, 12);
            else                          WS2812_Ctrl(0, 0, 0);
            break;
        default:                 WS2812_Ctrl(0, 0, 0);    break;
    }
}

void KeyTask_Entry(void const * argument)
{
    (void)argument;
    BSP_Buzzer_Init();
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_val, 2);
    WS2812_Ctrl(0, 8, 0);

    for (;;) {
        uint8_t st = (uint8_t)arm_ctrl.teach.state;
        uint32_t now = HAL_GetTick();

        vbus = (adc_val[0] * 3.3f / 65535.0f) * 11.0f;
        led_show(st);

        /* 状态变化：短鸣 + 打印一次 */
        if (st != s_last_state) {
            s_last_state = st;
            BSP_Buzzer_On();
            s_beep_off_tick = now + 60U;
        }
        if (s_beep_off_tick != 0U && (int32_t)(now - s_beep_off_tick) >= 0) {
            BSP_Buzzer_Off();
            s_beep_off_tick = 0U;
        }

        /* 1Hz 总线健康行：rx 不涨、off 一直涨、tec 冲高、bo=1 都一眼能看出来。
           tx 不说明问题：没人应答时它照样涨（FDCAN 配的是不重传，发一次算一帧） */
        if (now - s_can_tick >= 1000U) {
            bsp_can_stat_t a, b;
            s_can_tick = now;
            bsp_can_stat_get(CAN_CHANNEL_1, &a);
            bsp_can_stat_get(CAN_CHANNEL_2, &b);
            dbg_print("[CAN] c1 tx=%lu rx=%lu tec=%u rec=%u bo=%u off=%lu fix=%lu | "
                      "c2 tx=%lu rx=%lu tec=%u rec=%u bo=%u off=%lu fix=%lu\r\n",
                      (unsigned long)a.tx_ok, (unsigned long)a.rx, a.tec, a.rec, a.bo,
                      (unsigned long)a.bus_off, (unsigned long)a.recovered,
                      (unsigned long)b.tx_ok, (unsigned long)b.rx, b.tec, b.rec, b.bo,
                      (unsigned long)b.bus_off, (unsigned long)b.recovered);
        }

        osDelay(100);
    }
}
