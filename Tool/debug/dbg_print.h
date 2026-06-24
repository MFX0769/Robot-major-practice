/**
  * @file    dbg_print.h
  * @brief   简单调试打印工具
  * @note    通过USART6输出调试信息到VOFA
  */

#ifndef __DBG_PRINT_H__
#define __DBG_PRINT_H__

#include "usart.h"
#include <stdio.h>
#include <string.h>

/* 调试输出串口 (USART2，原USART6，调试改接U2) */
#define DBG_HUART huart2

/* 使能/禁用调试输出 */
#define DBG_ENABLE  1

#if DBG_ENABLE
    #define DBG_PRINT(fmt, ...)  dbg_print(fmt, ##__VA_ARGS__)
    #define DBG_LOG(tag, msg)    dbg_log(tag, msg)
#else
    #define DBG_PRINT(fmt, ...)
    #define DBG_LOG(tag, msg)
#endif

/* 函数声明 */
void dbg_init(void);
int dbg_print(const char *fmt, ...);
void dbg_log(const char *tag, const char *msg);
void dbg_hex(const char *tag, const uint8_t *data, uint16_t len);

#endif
