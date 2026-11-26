/**
  * @file    dbg_print.h
  * @brief   非阻塞调试打印（UART10 主通道 + USB CDC 尽力而为）
  */
#ifndef __DBG_PRINT_H__
#define __DBG_PRINT_H__

#include <stdint.h>

void dbg_init(void);
int  dbg_print(const char *fmt, ...);
void dbg_log(const char *tag, const char *msg);
void dbg_hex(const char *tag, const uint8_t *data, uint16_t len);
void dbg_task(void const *argu);
/* 日志链健康：入队/丢弃/截断/UART 发送/UART 超时。drop 增长 = 打印过量，不阻塞控制 */
void dbg_stats_get(uint32_t *enq, uint32_t *drop, uint32_t *trunc, uint32_t *tx, uint32_t *timeout);

#endif
