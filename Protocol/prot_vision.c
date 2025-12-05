/**
  * @file prot_vision.c
  * @version 1.1
  * @date 2025 12 15
	*
  * @brief  视觉信息解算
	*
  *	@author
  *
  */
#include "prot_vision.h"
#include "control_def.h"
#include "string.h"
#include "math.h"
#include "prot_T_imu.h"
#include "KalmanFilter.h"
#include "math_calcu.h"
#include "mode_switch_task.h"
#include "math.h"
#include "us_tim.h"
#include "prot_dr16.h"
#include "prot_new_rc.h"
#include "prot_rm_judge.h"
#include "prot_dr16.h"

#define ABS(x)  ((x)>0?(x):-(x))
#define PI      3.14159265358979323846f

/* 唯一全局变量 */
vision_t vision;
vision_tx_t vision_tx;
vision_mode_e vision_mode;
/*调试数值*/
uint8_t vis_e[2];
/*  NAN 保护宏  */
#define NAN_PROCESS(now, last)  \
    do { if (isnan(now)) now = last; else last = now; } while (0)

/* ----------------------------------------------------------------------- */
/* ① 串口回调：收包→落盘→限幅→射击标志                                     */
/* ----------------------------------------------------------------------- */
//void vision_data_handler(uint8_t *Vision_Data)
//{
//    static vision_aim_status_e last_aim_status;

//    /* 1. 拷帧 */
//    memcpy(vision.rx[0].buff, Vision_Data, VISION_DATA_LEN);

//    /* 2. 帧尾 & 重复帧检查 */
//    if (vision.rx[0].data.eof != 0xCC){
//        vision.rx_status = TAIL_ERROR; return;
//    }
//    if (vision.rx[0].data.cnt == vision.rx[1].data.cnt){
//        if (++vision.rx_repeat_cnt > 10){ vision.rx_status = REPEAT_ERROR; return; }
//    }
//    vision.online = 1;
//    vision.rx_status = NORMAL;
//    vision.rx_repeat_cnt = 0;
//    vision.rx[1].data.cnt = vision.rx[0].data.cnt;
//    vision.trace_id         = vision.rx[0].data.trace_id;

//    /* 3. NAN 保护 */
//    #define NAN_PROCESS(now, last)  \
//        do { if (isnan(now)) now = last; else last = now; } while (0)

//    NAN_PROCESS(vision.rx[0].data.yaw,  vision.rx[1].data.yaw);
//    NAN_PROCESS(vision.rx[0].data.pit,  vision.rx[1].data.pit);
//    NAN_PROCESS(vision.rx[0].data.dis,  vision.rx[1].data.dis);
//    NAN_PROCESS(vision.rx[0].data.fire, vision.rx[1].data.fire);
//    NAN_PROCESS(vision.rx[0].data.pos,  vision.rx[1].data.pos);

//    /* 4. 目标有效 */
//    if (vision.rx[0].data.aim_flag){
//        vision.aim_status        = AIMING;
//        vision.new_frame_flag    = 1;
//        vision.target_yaw_angle  = vision.rx[0].data.yaw;   // 度
//        vision.target_pit_angle  = -vision.rx[0].data.pit;  // 坐标系对齐
//        vision.yaw_min_err       = vision.rx[0].data.dis;
//        vision.pit_min_err       = vision.rx[0].data.dis2;

//        /* 简易限幅（度） */
//        if (vision.target_pit_angle < -23.0f)
//            vision.target_pit_angle = -23.0f;
//        if (fabs(vision.target_yaw_angle - gimbal.yaw_angle.fdb * 180.0f / PI) > 99.13f)
//            vision.target_yaw_angle = gimbal.yaw_angle.fdb * 180.0f / PI;

//        /* 射击判定 */
//        float yaw_err = fabs(gimbal.yaw_angle.fdb * 180.0f / PI - vision.target_yaw_angle);
//        float pit_err = fabs(gimbal.pit_angle.fdb * 180.0f / PI - vision.target_pit_angle);
//        vision.shoot_enable = (yaw_err < vision.yaw_min_err &&
//                               pit_err < vision.pit_min_err &&
//                               (vision.rx[0].data.fire == 1 || vision.rx[0].data.fire == 2));
//    }else{
//        vision.aim_status   = UNAIMING;
//        vision.shoot_enable = 0;
//    }

//    /* 首次丢失 */
//    if (last_aim_status == AIMING && vision.aim_status == UNAIMING)
//        vision.aim_status = FIRST_LOST;
//    last_aim_status = vision.aim_status;
//}

void vision_data_handler(uint8_t *Vision_Data)
{
    static vision_aim_status_e last_aim_status;

    /* 1. 拷帧 */
    memcpy(vision.rx[0].buff, Vision_Data, VISION_DATA_LEN);

    /* 2. 帧尾 & 重复帧检查 */
    if (vision.rx[0].data.eof != 0xCC) {
        vision.rx_status = TAIL_ERROR;
        return;
    }
    if (vision.rx[0].data.cnt == vision.rx[1].data.cnt) {
        if (++vision.rx_repeat_cnt > 10) {
            vision.rx_status = REPEAT_ERROR;
            return;
        }
    }
    vision.online           = 1;
    vision.rx_status        = NORMAL;
    vision.rx_repeat_cnt    = 0;
    vision.rx[1].data.cnt   = vision.rx[0].data.cnt;
    vision.trace_id         = vision.rx[0].data.trace_id;

    /* 3. NAN 保护 */
    NAN_PROCESS(vision.rx[0].data.yaw,  vision.rx[1].data.yaw);
    NAN_PROCESS(vision.rx[0].data.pit,  vision.rx[1].data.pit);
    NAN_PROCESS(vision.rx[0].data.dis,  vision.rx[1].data.dis);
    NAN_PROCESS(vision.rx[0].data.fire, vision.rx[1].data.fire);
    NAN_PROCESS(vision.rx[0].data.pos,  vision.rx[1].data.pos);

    /* 4. 目标有效 */
    if (vision.rx[0].data.aim_flag) {
        vision.aim_status     = AIMING;
        vision.new_frame_flag = 1;

        /* 4.1 保存视觉原始角度（单位保持视觉帧本身，度） */
        vision.target_yaw_angle = vision.rx[0].data.yaw;
        vision.target_pit_angle = vision.rx[0].data.pit;

        /* 4.2 限幅——默认关闭，用时打开 */
        // if (vsn_drive.target_pit_angle < -23.0f)
        //     vsn_drive.target_pit_angle = -23.0f;
        // if (fabs(vsn_drive.target_yaw_angle - gimbal.pid.yaw_angle_ref * 180.0f / PI) > 99.13f)
        //     vsn_drive.target_yaw_angle = gimbal.pid.yaw_angle_ref * 180.0f / PI;

        /* 4.3 保存视觉下发的误差门限（单位：弧度） */
        vision.yaw_min_err = vision.rx[0].data.dis;   // 视觉包 dis 已是弧度
        vision.pit_min_err = vision.rx[0].data.dis2;  // 视觉包 dis2 已是弧度

        /* 4.4 射击判定：PID 参考 vs 反馈，单位统一为弧度 */
        float yaw_err = fabsf(gimbal.yaw_angle.ref - gimbal.yaw_angle.fdb);
        float pit_err = fabsf(gimbal.pit_angle.ref  - gimbal.pit_angle.fdb);

        if (yaw_err < vision.yaw_min_err &&
            pit_err < vision.pit_min_err &&
            (vision.rx[0].data.fire == 1 || vision.rx[0].data.fire == 2))
        {
            vision.shoot_enable = 1;
        } else {
            vision.shoot_enable = 0;
        }
    } else {
        vision.aim_status   = UNAIMING;
        vision.shoot_enable = 0;
    }

    /* 5. 调试标志位（与旧代码一致） */
    vis_e[0] = (ABS(gimbal.yaw_angle.fdb- vision.target_yaw_angle * PI / 180.0f) < vision.yaw_min_err) ? 1 : 0;
    vis_e[1] = (ABS(gimbal.pit_angle.fdb  - vision.target_pit_angle * PI / 180.0f) < vision.pit_min_err) ? 1 : 0;

    /* 6. 首次丢失 */
    if (last_aim_status == AIMING && vision.aim_status == UNAIMING)
        vision.aim_status = FIRST_LOST;

    last_aim_status = vision.aim_status;
}

/* ----------------------------------------------------------------------- */
/* ② 1 ms 任务：给云台参考                                                 */
/* ----------------------------------------------------------------------- */
void vsn_gimbal_ref_calc(void)
{
    switch (vision.aim_status){
    case AIMING:
        if (vision.new_frame_flag){
            gimbal.pit_angle.ref = vision.target_pit_angle * PI / 180.0f;
            gimbal.yaw_angle.ref = vision.target_yaw_angle * PI / 180.0f;
            vision.new_frame_flag = 0;
        }
        break;

    case FIRST_LOST:
        vision.aim_status = UNAIMING;
        gimbal.pit_angle.ref = gimbal.pit_angle.fdb;
        gimbal.yaw_angle.ref = gimbal.yaw_angle.fdb;
        break;

    case UNAIMING:
        /* 丢失目标：遥控增量 */
        if (rc.sw1 == RC_DN){
            gimbal.yaw_angle.ref += (-rc.mouse.x) * KEYBOARD_SCALE_YAW;
            gimbal.pit_angle.ref += (-rc.mouse.y) * KEYBOARD_SCALE_PIT;
        }else{
            gimbal.yaw_angle.ref += rc.ch1 * scale.ch1 * (-1);
            gimbal.pit_angle.ref += rc.ch2 * scale.ch2 * (-1);
        }
        break;
    }
}

//void Trace_ID_Select_NEW(new_rc_info_t *new_rc)
//{
//	if(new_rc->key_W)  vision_tx.data.trace_id  = 7;
//	if(new_rc->key_A)  vision_tx.data.trace_id  = 3;
//	if(new_rc->key_Z)  vision_tx.data.trace_id  = 4;
//	if(new_rc->key_D)  vision_tx.data.trace_id  = 1;
//	if(new_rc->key_S)  vision_tx.data.trace_id  = 2;	
//}

void Trace_ID_Select_OLD(rc_info_t *rc)
{
	if(rc->kb.bit.W)  vision_tx.data.trace_id  = 7;
	if(rc->kb.bit.Z)  vision_tx.data.trace_id  = 4;
	if(rc->kb.bit.A)  vision_tx.data.trace_id  = 3;
	if(rc->kb.bit.D)  vision_tx.data.trace_id  = 1;
	if(rc->kb.bit.S)  vision_tx.data.trace_id  = 2;	
}

//任务里调用函数即可发送
void vision_output_data(void)
{
    /* 1. 帧头/尾固定 */
    vision_tx.data.SOF  = 0x11;
    vision_tx.data.eof1 = 0x22;
    vision_tx.data.eof2 = 0x33;

    /* 2. 浮点→小端 4 字节（与旧代码符号一致） */
    float tmp = -imu_data.pitch;           memcpy(vision_tx.data.imu_pit,     &tmp, 4);
    tmp = -imu_data.yaw;                   memcpy(vision_tx.data.imu_yaw,     &tmp, 4);
    tmp = -imu_data.wy;                    memcpy(vision_tx.data.imu_pit_spd, &tmp, 4);
    tmp = -imu_data.wz;                    memcpy(vision_tx.data.imu_yaw_spd, &tmp, 4);
    tmp = Shoot_Data.bullet_speed;         memcpy(vision_tx.data.shoot_speed, &tmp, 4);

    /* 3. 位域+单字节 */
    vision_tx.data.vacancy       = 0;                       // 预留，填 0
    vision_tx.data.camp = 0;
    vision_tx.data.aiming_mode   = vision_mode;             // 0:自瞄 1:大能量 2:小能量 4:反陀螺
    vision_tx.data.shooter_speed = 3;                       // 固定 3 档
//    vision_tx.data.trace_id      = vision.trace_id;         // 来自视觉包

	Trace_ID_Select_OLD(&rc);
    /* 4. DMA 一次发出 */
//    HAL_UART_Transmit_DMA(&huart4, vision_tx.buff, 25);
}
