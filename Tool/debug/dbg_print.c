/**
  * @file    dbg_print.c
  * @brief   简单调试打印工具实现
  */

#include "dbg_print.h"
#include <stdio.h>
#include <stdarg.h>

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

    HAL_UART_Transmit(&DBG_HUART, (uint8_t *)dbg_buf, len, 10);
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
