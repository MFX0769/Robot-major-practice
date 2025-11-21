/**
  * @file prot_vision.c
  * @version 1.1
  * @date 2022 7 10
	*
  * @brief  视觉信息解算
	*
  *	@author
  *
  */
#include "prot_vision.h"
#include "control_def.h"
#include "can_comm.h"
#include "string.h"
#include "math.h"
#include "prot_T_imu.h"
#include "KalmanFilter.h"
#include "math_calcu.h"
//#include "modeswitch_task.h"
#include "math.h"
#include "ubf.h"
#include "us_tim.h"
#include "prot_dr16.h"
#include "prot_new_rc.h"

#define ABS(x)  ((x)>0?(x):-(x))
#define PI      3.14159265358979323846f

/* 唯一全局变量 */
vsn_drive_t vsn_drive = {0};

///*  NAN 保护宏  */
//#define NAN_PROCESS(now, last)  \
//    do { if (isnan(now)) now = last; else last = now; } while (0)

///* ----------------------------------------------------------------------- */
///* ① 串口回调：收包→落盘→限幅→射击标志                                     */
///* ----------------------------------------------------------------------- */
//void vision_data_handler(uint8_t *Vision_Data)
//{
//    static vision_aim_status_e last_aim_status;

//    memcpy(vsn_drive.rx[0].buff, Vision_Data, VISION_DATA_LEN);

//    /* 帧尾 & 重复帧检查 */
//    if (vsn_drive.rx[0].data.eof != 0xCC){
//        vsn_drive.rx_status = TAIL_ERROR; return;
//    }
//    if (vsn_drive.rx[0].data.cnt == vsn_drive.rx[1].data.cnt){
//        if (++vsn_drive.rx_repeat_cnt > 10){ vsn_drive.rx_status = REPEAT_ERROR; return; }
//    }
//    vsn_drive.online = 1;
//    vsn_drive.rx_status = NORMAL;
//    vsn_drive.rx_repeat_cnt = 0;
//    vsn_drive.rx[1].data.cnt = vsn_drive.rx[0].data.cnt;
//    vsn_drive.trace_id       = vsn_drive.rx[0].data.trace_id;

//    NAN_PROCESS(vsn_drive.rx[0].data.yaw,  vsn_drive.rx[1].data.yaw);
//    NAN_PROCESS(vsn_drive.rx[0].data.pit,  vsn_drive.rx[1].data.pit);
//    NAN_PROCESS(vsn_drive.rx[0].data.dis,  vsn_drive.rx[1].data.dis);
//    NAN_PROCESS(vsn_drive.rx[0].data.fire, vsn_drive.rx[1].data.fire);
//    NAN_PROCESS(vsn_drive.rx[0].data.pos,  vsn_drive.rx[1].data.pos);

//    if (vsn_drive.rx[0].data.aim_flag){
//        vsn_drive.aim_status = AIMING;
//        vsn_drive.new_frame_flag = 1;

//        /* 1. 落盘：度 → 直接存 */
//        vsn_drive.target_yaw_angle = vsn_drive.rx[0].data.yaw;
//        vsn_drive.target_pit_angle = -vsn_drive.rx[0].data.pit;   // 坐标系对齐
//        vsn_drive.yaw_min_err      = vsn_drive.rx[0].data.dis;
//        vsn_drive.pit_min_err      = vsn_drive.rx[0].data.dis2;

//        /* 2. 简易限幅（度） */
//        if (vsn_drive.target_pit_angle < -23.0f)
//            vsn_drive.target_pit_angle = -23.0f;
//        if (fabs(vsn_drive.target_yaw_angle - gimbal.yaw_angle.fdb * 180.0f / PI) > 99.13f)
//            vsn_drive.target_yaw_angle = gimbal.yaw_angle.fdb * 180.0f / PI;

//        /* 3. 射击允许：误差小 + 视觉让开火 */
//        float yaw_err = fabs(gimbal.yaw_angle.fdb * 180.0f / PI - vsn_drive.target_yaw_angle);
//        float pit_err = fabs(gimbal.pit_angle.fdb * 180.0f / PI - vsn_drive.target_pit_angle);
//        vsn_drive.shoot_enable = (yaw_err < vsn_drive.yaw_min_err &&
//                                  pit_err < vsn_drive.pit_min_err &&
//                                  (vsn_drive.rx[0].data.fire == 1 || vsn_drive.rx[0].data.fire == 2));
//    }else{
//        vsn_drive.aim_status   = UNAIMING;
//        vsn_drive.shoot_enable = 0;
//    }

//    /* 首次丢失标记 */
//    if (last_aim_status == AIMING && vsn_drive.aim_status == UNAIMING)
//        vsn_drive.aim_status = FIRST_LOST;
//    last_aim_status = vsn_drive.aim_status;
//}

///* ----------------------------------------------------------------------- */
///* ② 1 ms 任务：给云台参考                                                 */
///* ----------------------------------------------------------------------- */
//void vsn_gimbal_ref_calc(void)
//{
//    switch (vsn_drive.aim_status){
//    case AIMING:
//        if (vsn_drive.new_frame_flag){                      // ① 只用新包
//            gimbal.pid.pit_angle_ref = vsn_drive.target_pit_angle * PI / 180.0f;
//            gimbal.pid.yaw_angle_ref = vsn_drive.target_yaw_angle * PI / 180.0f;
//            vsn_drive.new_frame_flag = 0;                   // ② 立即清零
//        }
//        break;

//    case FIRST_LOST:
//        vsn_drive.aim_status = UNAIMING;                    // ③ 直接改接收结构体
//        gimbal.pid.pit_angle_ref = gimbal.pit_angle.fdb;
//        gimbal.pid.yaw_angle_ref = gimbal.yaw_angle.fdb;
//        break;
//    case UNAIMING:
//        /* 丢失目标：遥控增量（照搬你旧代码） */
//            if (rc.sw1 == RC_DN){                       // 旧键鼠
//                gimbal.pid.yaw_angle_ref += (-rc.mouse.x) * KEYBOARD_SCALE_YAW;
//                gimbal.pid.pit_angle_ref += (-rc.mouse.y) * KEYBOARD_SCALE_PIT;
//            }else{                                      // 旧摇杆
//                gimbal.pid.yaw_angle_ref += rc.ch1 * scale.ch1 * (-1);
//                gimbal.pid.pit_angle_ref += rc.ch2 * scale.ch2 * (-1);
//            }
//        
//        break;
//    }
//}
