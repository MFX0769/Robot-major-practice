/**
  * @file    dbg_print.c
  * @brief   简单调试打印工具实现
  */

#include "dbg_print.h"
#include <stdio.h>
#include <stdarg.h>
#include "usbd_cdc_if.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"   /* 达妙板调试走板载 USB CDC (2026-11-22) */

static char dbg_buf[256];

/**
  * @brief  初始化调试输出
  */
void dbg_init(void)
{
    memset(dbg_buf, 0, sizeof(dbg_buf));
    dbg_print("\r\n=== System Boot ===\r\n");
}

/**
  * @brief  格式化打印
  */
int dbg_print(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(dbg_buf, sizeof(dbg_buf), fmt, args);
    va_end(args);

    /* 双路输出 (2026-11-22): 板载 USB CDC + USART10(PE3, DAP VCP) */
    {
        extern UART_HandleTypeDef huart10;
        HAL_UART_Transmit(&huart10, (uint8_t *)dbg_buf, (uint16_t)len, 20);  /* U7 freed for line sensor */
    }
    /* CDC 仅在调度器已启动后使用 (osDelay 在 main 阶段非法) */
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        usb_tx_send((uint8_t *)dbg_buf, (uint16_t)len);
    }
    return len;
}

/**
  * @brief  带标签的日志
  */
void dbg_log(const char *tag, const char *msg)
{
    dbg_print("[%s] %s\r\n", tag, msg);
}

/**
  * @brief  打印十六进制数据
  */
void dbg_hex(const char *tag, const uint8_t *data, uint16_t len)
{
    dbg_print("[%s] HEX(%d): ", tag, len);
    for (uint16_t i = 0; i < len && i < 32; i++) {
        dbg_print("%02X ", data[i]);
    }
    dbg_print("\r\n");
}
