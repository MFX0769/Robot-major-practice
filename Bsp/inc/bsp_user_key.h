/**
  * @file    bsp_user_key.h
  * @brief   PA15 单按钮事件扫描（短按/双击/三击/长按2s/长按5s）
  */
#ifndef __BSP_USER_KEY_H__
#define __BSP_USER_KEY_H__

#include "main.h"
#include <stdint.h>

typedef enum {
    KEY_EVT_NONE = 0,
    KEY_EVT_SHORT,      /* 单击（300ms 判定窗后上报） */
    KEY_EVT_DOUBLE,     /* 双击 */
    KEY_EVT_TRIPLE,     /* 三击及以上 */
    KEY_EVT_LONG2,      /* 按住 2~5s，松开时上报 */
    KEY_EVT_LONG5       /* 按住满 5s，立即上报 */
} key_event_e;

void        BSP_UserKey_Init(void);
key_event_e BSP_UserKey_Poll(void);   /* 建议 3ms 调用一次 */

#endif
