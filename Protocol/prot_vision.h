/**
  * @file  prot_vision.h
  * @brief 视觉串口接收 + 云台参考输出（最小版）
  */

#ifndef __PROT_VISION_H__
#define __PROT_VISION_H__
#include <stdint.h>
#include <math.h>

#define VISION_DATA_LEN  28

/* 视觉侧模式 */
typedef enum {
    vMODE_AUTO        = 0,    /* 自瞄 */
    vMODE_bENERGY     = 1,    /* 大能量机关 */
    vMODE_sENERGY     = 2,    /* 小能量机关 */
    vMODE_ANTISPIN    = 4,    /* 中远距离反小陀螺模式 */
} vision_mode_e;

/* 视觉状态 */
typedef enum {
    UNAIMING   = 0,
    AIMING     = 1,
    FIRST_LOST = 2,
} vision_aim_status_e;

typedef enum {
    NORMAL      = 0,
    TAIL_ERROR  = 1,
    REPEAT_ERROR= 2,
} vision_rx_status_e;

/* 28 B 视觉帧 */
typedef struct {
    uint32_t rx_repeat_cnt;
    vision_rx_status_e  rx_status;
    vision_aim_status_e aim_status;
    uint8_t  online;
    uint8_t  shoot_enable;
    uint8_t  new_frame_flag;
    uint8_t  trace_id;

    float target_yaw_angle; // 度
    float target_pit_angle; // 度
    float yaw_min_err;      // 度
    float pit_min_err;      // 度

    union {
        uint8_t buff[VISION_DATA_LEN];
        struct {
            float yaw,pit,dis,dis2,fire,pos;
            uint8_t empty,cnt:6,ist_flag:1,aim_flag:1,trace_id,eof;
        } data;
    } rx[2];
} vsn_drive_t;

/* 全局变量 */
extern vsn_drive_t vsn_drive;

/* 函数 */
void vision_data_handler(uint8_t *data);   // 串口回调
void vsn_gimbal_ref_calc(void);            // 1 ms 任务
#endif 
