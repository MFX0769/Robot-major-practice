/**
  * @file     prot_sbus.c
  * @brief    WBUS/SBUS 最小解析实现
  *           - 解析 25 字节标准 SBUS 帧，提取 16 路 11bit 通道值
  *           - 更新接收计数 `wbus_rx_cnt`，并用系统节拍计算 `wbus_rx_freq`
  */

/* Includes ------------------------------------------------------------------*/
#include "prot_sbus.h"
#include "cmsis_os.h"
#include "string.h"

/* 全局帧实例与统计 */
wbus_frame_u wbus_rc;
volatile uint32_t wbus_rx_cnt = 0;
volatile float    wbus_rx_freq = 0.0f;

/* 解析函数：把原始 buf 解包到 rc_frame->data 中（不做复杂校验） */
void sbus_parse_frame(wbus_frame_u *rc_frame)
{
  uint16_t ch[16] = {0};
  uint8_t *buff;

  if (rc_frame == NULL) return;
  buff = rc_frame->buf;

  /* 简单帧头校验 */
  if (buff[0] != 0x0F) return;

  ch[0]  = (buff[1]  | buff[2]  << 8) & 0x07FF;
  ch[1]  = (buff[2] >> 3 | buff[3] << 5) & 0x07FF;
  ch[2]  = (buff[3] >> 6 | buff[4] << 2 | buff[5] << 10) & 0x07FF;
  ch[3]  = (buff[5] >> 1 | buff[6] << 7) & 0x07FF;
  ch[4]  = (buff[6] >> 4 | buff[7] << 4) & 0x07FF;
  ch[5]  = (buff[7] >> 7 | buff[8] << 1 | buff[9] << 9) & 0x07FF;
  ch[6]  = (buff[9] >> 2 | buff[10] << 6) & 0x07FF;
  ch[7]  = (buff[10] >> 5 | buff[11] << 3) & 0x07FF;
  ch[8]  = (buff[12] | buff[13] << 8) & 0x07FF;
  ch[9]  = (buff[13] >> 3 | buff[14] << 5) & 0x07FF;
  ch[10] = (buff[14] >> 6 | buff[15] << 2 | buff[16] << 10) & 0x07FF;
  ch[11] = (buff[16] >> 1 | buff[17] << 7) & 0x07FF;
  ch[12] = (buff[17] >> 4 | buff[18] << 4) & 0x07FF;
  ch[13] = (buff[18] >> 7 | buff[19] << 1 | buff[20] << 9) & 0x07FF;
  ch[14] = (buff[20] >> 2 | buff[21] << 6) & 0x07FF;
  ch[15] = (buff[21] >> 5 | buff[22] << 3) & 0x07FF;

  
  /* 简单无效检测：若所有通道均为 0，则视为无效并清零 */
  bool all_zero = true;
  for (int i = 0; i < 16; i++) { if (ch[i] != 0) { all_zero = false; break; } }
  if (all_zero)
  {
    memset(&rc_frame->data, 0, sizeof(wbus_rc_data));
    return;
  }

  /* 写回解码结果到 data（与 buf 是 union，此时 buff 已经读完，可以安全覆盖） */
  rc_frame->data.ch1  = (int16_t)ch[0]  - 1024;
  rc_frame->data.ch2  = (int16_t)ch[1]  - 1024;
  rc_frame->data.ch3  = (int16_t)ch[2]  - 1024;
  rc_frame->data.ch4  = (int16_t)ch[3]  - 1024;
  rc_frame->data.ch5  = (int16_t)ch[4]  - 1024;
  rc_frame->data.ch6  = (int16_t)ch[5]  - 1024;
  rc_frame->data.ch7  = (int16_t)ch[6]  - 1024;
  rc_frame->data.ch8  = (int16_t)ch[7]  - 1024;
  rc_frame->data.ch9  = (int16_t)ch[8]  - 1024;
  rc_frame->data.ch10 = (int16_t)ch[9]  - 1024;
  rc_frame->data.ch11 = (int16_t)ch[10] - 1024;
  rc_frame->data.ch12 = (int16_t)ch[11] - 1024;
  rc_frame->data.ch13 = (int16_t)ch[12] - 1024;
  rc_frame->data.ch14 = (int16_t)ch[13] - 1024;
  rc_frame->data.ch15 = (int16_t)ch[14] - 1024;
  rc_frame->data.ch16 = (int16_t)ch[15] - 1024;

  /* 拨杆状态：实测拨杆接在ch6(ch[5])，不是ch5(ch[4])。实测原始值：最上raw~353(居中后-671)，
     中间raw~1024(居中后0)，最下未测但按对称估计raw~1695 */
  if (ch[5] <= SBUS_SW_DOWN_THR) rc_frame->data.sw1 = 2;       /* 拨杆最上 */
  else if (ch[5] >= SBUS_SW_UP_THR) rc_frame->data.sw1 = 0;    /* 拨杆最下 */
  else rc_frame->data.sw1 = 1;                                   /* 拨杆中间 */

  /* 更新接收统计：帧计数与频率（用系统节拍计算，自上电/内核启动以来的平均） */
  static uint32_t last_tick = 0;
  uint32_t now_tick = osKernelSysTick();
  if (last_tick != 0 && now_tick > last_tick)
  {
    wbus_rx_freq = 1000.0f / (float)(now_tick - last_tick);
  }
  last_tick = now_tick;
  wbus_rx_cnt++;
}

/* 兼容回调：复制到全局并解析 */
void sbus_callback_handler(uint8_t *buff, uint16_t len)
{
  if (buff == NULL || len < SBUS_FRAME_SIZE) return;
  memcpy(wbus_rc.buf, buff, SBUS_FRAME_SIZE);
  sbus_parse_frame(&wbus_rc);
}

