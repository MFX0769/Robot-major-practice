/**
  * @file    debug_task.c
  * @brief   VOFA+ 示波调试通道（USART6）
  * @note    2026-10-25 大清洗：删除旧比赛车（云台/摩擦轮/发射机构/视觉）的示波通道
  *          case 1~10，仅保留课设小车在用的 case 11/12/13。历史通道在 git 历史里。
  */
#include "debug_task.h"
#include "config.h"
#include "cmsis_os.h"
#include "data_log.h"
#include "prot_rm_motor.h"
#include "prot_sbus.h"
#include "prot_8irtraking.h"

extern dji_motor_t chassis_2006[4];

/* 波形通道选择：11=底盘电机ecd+SBUS前4通道 12=SBUS全部通道 13=巡线位置 */
uint8_t debug_wave = 13;

/* 示波数据打包（由 log_scope_data_output 调用） */
void log_scope_data_pkg(void)
{
    switch (debug_wave) {
        case 11: { /* 底盘电机ecd + SBUS四通道 */
            log_scope_get_data(chassis_2006[0].ecd);
            log_scope_get_data(chassis_2006[1].ecd);
            log_scope_get_data(chassis_2006[2].ecd);
            log_scope_get_data(chassis_2006[3].ecd);
            log_scope_get_data(wbus_rc.data.ch1);
            log_scope_get_data(wbus_rc.data.ch2);
            log_scope_get_data(wbus_rc.data.ch3);
            log_scope_get_data(wbus_rc.data.ch4);
            break;
        }
        case 12: { /* SBUS遥控器全部通道 */
            log_scope_get_data(wbus_rc.data.ch1);
            log_scope_get_data(wbus_rc.data.ch2);
            log_scope_get_data(wbus_rc.data.ch3);
            log_scope_get_data(wbus_rc.data.ch4);
            log_scope_get_data(wbus_rc.data.ch5);
            log_scope_get_data(wbus_rc.data.ch6);
            log_scope_get_data(wbus_rc.data.ch7);
            log_scope_get_data(wbus_rc.data.ch8);
            log_scope_get_data(wbus_rc.data.sw1);
            log_scope_get_data(wbus_rx_cnt);
            log_scope_get_data(wbus_rx_freq);
            break;
        }
        case 13: { /* 巡线模块算出来的线位置 */
            const prot_8irtraking_data_t *line_data = prot_8irtraking_get_data();
            log_scope_get_data(line_data->state);
            log_scope_get_data(line_data->position);
            break;
        }
        default: {
            break;
        }
    }
}

/* VOFA+ 数据发送任务 */
void debug_task(void const* argu)
{
    uint32_t thread_wake_time = osKernelSysTick();
    for(;;)
    {
        thread_wake_time = osKernelSysTick();
#if DEBUG_VOFA_ENABLE
        log_scope_data_output();
#endif

        osDelayUntil(&thread_wake_time, 5);
    }
}
