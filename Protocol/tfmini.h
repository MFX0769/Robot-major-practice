/**
  * @file tfmini.h
  * @brief 北醒 TF-mini Plus 前向测距：UART9_RX = PD14（扩展排针 11 脚），115200-8N1，9 字节帧
  * @note 59 59 distL distH strL strH tempL tempH sum（前 8 字节求和的低 8 位 == 第 9 字节）。
  *       距离单位出厂是 cm（0~1200）；模块被改成 mm 的话用 'avoid unit mm' 告诉避障逻辑。
  *       中断里逐字节解析，不依赖帧间隔；100ms 没有新帧算离线。
  */
#ifndef TFMINI_H
#define TFMINI_H

#include <stdint.h>

#define TFMINI_NUM            1U      /* 现在只接前向一个；要加再扩 */
#define TFMINI_FRAME_LEN      9U
#define TFMINI_MAX_AGE_MS     100U
#define TFMINI_HDR            0x59U
#define TFMINI_STRENGTH_MIN   20U     /* 官方手册：低于 20 时距离输出 FFFF，不可信 */

typedef enum {
    TFMINI_FRONT = 0,
} tfmini_id_e;

typedef struct {
    uint16_t dist_raw;      /* 帧里的原始距离，单位 cm 或 mm，见 'avoid unit' */
    uint16_t strength;
    int16_t temp_c;
    uint32_t last_tick;     /* 最近一帧校验通过的时刻 */
    uint32_t rx_frames;     /* 校验通过的帧数 */
    uint32_t rx_bytes;
    uint32_t crc_errors;
} tfmini_data_t;

extern tfmini_data_t tfmini[TFMINI_NUM];

void tfmini_init(void);
uint8_t tfmini_online(uint8_t id);       /* 100ms 内收到过帧 */
uint16_t tfmini_distance(uint8_t id);    /* 原始距离；离线返回 0 */
uint8_t tfmini_frame_valid(uint8_t id);  /* 最近一帧的强度和距离都在可信范围 */
void tfmini_rx_isr(void);                /* UART9_IRQHandler 调 */

#endif
