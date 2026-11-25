/**
  * @file    log_task.c
  * @brief   日志任务（壳）
  * @note    2026-10-25 大清洗：删除旧空中机器人的裁判系统/摩擦轮/拨盘日志尸骸（原本全部
  *          是注释代码）。文本日志走 dbg_print（DEBUG_TEXT_ENABLE），波形走 VOFA+
  *          （debug_task）。本任务当前未在 freertos.c 注册，保留函数以备启用。
  */
#include "log_task.h"
#include "cmsis_os.h"

/**
  * @brief  日志任务：目前无周期性日志输出，仅维持任务节拍
  */
void AirRobot_Log_Task(const void* argu)
{
    uint32_t thread_wake_time = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&thread_wake_time, 10);
    }
}
