/**
  ******************************************************************************
  * @file    debug_port.h
  * @author  taurus
  * @brief   Unified debug output port: USB CDC (board USB) or DAP VCP (wireless).
  * @attention
  *   Select route by DEBUG_PORT_* macro (2026-11-22).
  *   - USB CDC  : onboard USB, appears as COM137 (STM32 VID_0483:5740). Wired only.
  *   - DAP VCP  : Horco CMSIS-DAP virtual COM (COM134), WIRELESS capable.
  *                Route through UART9 (PD14 RX / PD15 TX, from rm_main.ioc).
  ******************************************************************************
*/
#ifndef __DEBUG_PORT_H
#define __DEBUG_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 输出通道配置（可同时开两路，H7 性能足够）:
 *   DEBUG_PORT_USB_CDC : 板载 USB CDC (COM137)
 *   DEBUG_PORT_UART10  : USART10 PE3(TX)/PE2(RX) -> DAP VCP (用户已接线)
 *   两路可同时为 1 -> 双路打印
 */
#ifndef DEBUG_PORT_USB_CDC
#define DEBUG_PORT_USB_CDC   1
#endif
#ifndef DEBUG_PORT_UART10
#define DEBUG_PORT_UART10    1
#endif

/* 初始化 debug 端口 (USART10 已在 main.c MX_USART10_UART_Init) */
void debug_port_init(void);

/* 阻塞发送一段 buffer (带 CDC busy 重试) */
void debug_port_write(const uint8_t *buf, uint16_t len);

/* 格式化打印 (内部 vsnprintf, 单次上限 256 字节) */
void debug_printf(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* __DEBUG_PORT_H */
