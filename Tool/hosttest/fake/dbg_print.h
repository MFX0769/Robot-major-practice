#ifndef FAKE_DBG_PRINT_H
#define FAKE_DBG_PRINT_H
#include <stdint.h>
int dbg_print(const char *fmt, ...);
/* 测试台不编 dbg_task/USB 发送任务；给诊断统计一个确定值 */
void dbg_stats_get(uint32_t *enq, uint32_t *drop, uint32_t *trunc, uint32_t *tx, uint32_t *timeout);
#endif
