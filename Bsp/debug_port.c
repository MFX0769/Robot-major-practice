/**
  ******************************************************************************
  * @file    debug_port.c
  * @brief   Unified debug output implementation (USB CDC + USART10, can both ON)
  ******************************************************************************
*/
#include "debug_port.h"
#include "main.h"
#include "cmsis_os.h"
#include <stdarg.h>
#include <stdio.h>

#if DEBUG_PORT_USB_CDC
#include "usbd_cdc_if.h"     /* CDC_Transmit_HS */
#endif
#if DEBUG_PORT_UART10
#include "usart.h"           /* huart10 (USART10 PE3 TX / PE2 RX) */
#endif

void debug_port_init(void)
{
    /* 外设已在 main.c 初始化: MX_USB_DEVICE_Init (defaultTask) + MX_USART10_UART_Init */
}

/* 单路底层发送 */
static void dbg_usb_write(const uint8_t *buf, uint16_t len)
{
#if DEBUG_PORT_USB_CDC
    uint16_t retry = 0;
    while (CDC_Transmit_HS((uint8_t*)buf, len) == USBD_BUSY && retry < 20) {
        osDelay(1);
        retry++;
    }
#else
    (void)buf; (void)len;
#endif
}

static void dbg_uart10_write(const uint8_t *buf, uint16_t len)
{
#if DEBUG_PORT_UART10
    HAL_UART_Transmit(&huart10, (uint8_t*)buf, len, 100);
#else
    (void)buf; (void)len;
#endif
}

void debug_port_write(const uint8_t *buf, uint16_t len)
{
    dbg_usb_write(buf, len);
    dbg_uart10_write(buf, len);
}

void debug_printf(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) {
        if (n > (int)sizeof(buf)-1) n = (int)sizeof(buf)-1;
        debug_port_write((const uint8_t*)buf, (uint16_t)n);
    }
}
