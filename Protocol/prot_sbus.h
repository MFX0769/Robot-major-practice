/*
 * prot_sbus.h
 * WBUS/SBUS 最小解析头文件
 * - 提供最小化的帧解析与接收统计（rx_cnt, rx_freq）
 */

#ifndef __PROT_SBUS_H__
#define __PROT_SBUS_H__

#include "stdint.h"
#include "stdbool.h"
#include "string.h"

/* SBUS/WBUS 帧长度（标准 25 字节） */
#define SBUS_FRAME_SIZE 25

/* 开关位置判断阈值（宏定义，便于以后调整） */
#define SBUS_CH_MAX        2047
#define SBUS_CH_MID        1024
#define SBUS_SW_UP_THR     1500
#define SBUS_SW_DOWN_THR   600

/* 遥控数据结构：wbus_rc_data（只保留必要的通道数值以及兼容字段） */
typedef  struct
{
    int16_t ch1;   
    int16_t ch2;   
    int16_t ch3;  
    int16_t ch4;  

    int16_t ch5;  
    int16_t ch6;
    int16_t ch7;
    int16_t ch8;

    int16_t ch9;
    int16_t ch10;
    int16_t ch11;
    int16_t ch12;

    int16_t ch13;
    int16_t ch14;
    int16_t ch15;
    int16_t ch16;

    uint8_t sw1;   /* SWB(ch6) 拨杆上/中/下状态：0=下,1=中,2=上 */
    uint8_t sw2;   /* SWC(ch7) 拨杆上/中/下状态：0=下,1=中,2=上 */
    uint8_t sw3;   /* SWD(ch8) 两档开关：0/1，给舵机供电继电器单独用 */



} __attribute__((packed)) wbus_rc_data;

/* 联合体：方便直接把接收到的原始帧写入 buf，然后解析为 data */
typedef union
{
    uint8_t buf[SBUS_FRAME_SIZE];
    wbus_rc_data data;
} wbus_frame_u;

/* 全局实例（在 prot_sbus.c 中定义） */
extern wbus_frame_u wbus_rc;
extern volatile uint32_t wbus_rx_cnt;
extern volatile float    wbus_rx_freq;

/* 将一帧原始数据解析到 rc_frame->data 中（最小实现，不做复杂校验） */
void sbus_parse_frame(wbus_frame_u *rc_frame);

/* 兼容回调：传入原始 buf 与 len，内部复制到全局并解析 */
void sbus_callback_handler(uint8_t *buff, uint16_t len);

#endif


