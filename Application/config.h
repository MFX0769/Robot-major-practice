#ifndef __CONFIG_H__
#define __CONFIG_H__

/*=============================================================
 * 调试输出模式开关 (共用 USART6，二选一)
 *   DEBUG_TEXT_ENABLE = 1 : 文本调试打印 (dbg_print)
 *   DEBUG_VOFA_ENABLE = 1 : VOFA+ 波形输出 (log_scope_data_output)
 * 两者不能同时为1
 *=============================================================*/
#define DEBUG_TEXT_ENABLE   0
#define DEBUG_VOFA_ENABLE   1

#if DEBUG_TEXT_ENABLE && DEBUG_VOFA_ENABLE
  #error "DEBUG_TEXT_ENABLE 和 DEBUG_VOFA_ENABLE 不能同时使能 (共用USART6)"
#endif

/* 机械臂调试打印开关见 arm_task.h (ARM_DBG_ENABLE) */

#endif /* __CONFIG_H__ */
