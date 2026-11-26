#ifndef FAKE_USB_COMM_H
#define FAKE_USB_COMM_H
#include <stdint.h>
/* 真头文件依赖 USB 设备类型；测试台只需要诊断统计接口 */
void usb_tx_stats_get(uint32_t *enq, uint32_t *drop, uint32_t *tx, uint32_t *busy_drop);
#endif
