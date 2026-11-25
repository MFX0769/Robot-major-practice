#ifndef __CONFIG_H__
#define __CONFIG_H__

/*=============================================================
 * 调试输出模式开关 (2026-11-22 迁移到达妙 ALL 基座后更新)
 *   dbg_print -> 双路: 板载 USB CDC + USART10(PE3, DAP VCP)
 *   VOFA+     -> 板载 USB CDC (原厂 imu_task vofa)
 *   通道已分离, TEXT 与 VOFA 可同时为 1 (旧 U6 互斥约束解除)
 *=============================================================*/
#define DEBUG_TEXT_ENABLE   1
#define DEBUG_VOFA_ENABLE   0

/* 机械臂调试打印开关见 arm_task.h (ARM_DBG_ENABLE) */

#endif /* __CONFIG_H__ */
