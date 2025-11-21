///**
//  * @file gimbal_task.c
//  * @version 1.0
//  * @date  2025.11.19
//	*
//  * @brief 26赛季云台任务
//	*
//  *	@author  myh
//  *
//  */
//  
//#include "gimbal_task.h"
//#include "pid.h"
//#include "math_calcu.h"
//#include "remote_msg.h"
//#include "shoot_task.h"
//#include "mode_switch_task.h"
//#include "control_def.h"
//#include "comm_task.h"
//#include "can_comm.h"
//#include "status_task.h"
//#include "prot_T_imu.h"
//#include "KalmanFilter.h"
////#incldue "prot_vision.h"
//#include "prot_rm_motor.h"
//#include "prot_dm_motor.h"

//gimbal_t gimbal;

//static void gimbal_init(void)
//{
//    memset(&gimbal, 0, sizeof(gimbal_t));

////    pid_init(&gimbal.pit_angle.pid, NONE, 18, 0, 0, 0, 7);//20 0 0 有测速模块
////    pid_init(&gimbal.pit_spd.pid, NONE, -1.2f, -0.006f, 0, 0.4f, 2.2f);//-1.0 -0.008
////    
//	PID_struct_init(&gimbal.pit_angle.pid,  150,5.0f, 0.0f, 0.0f, 0.0f, 0,0,0,NONE_IMPROVE);//20 0 0 有测速模块
//    PID_struct_init(&gimbal.pit_spd.pid, 7, 1.0f, 0.0f,0.0f, 0.00, 0, 0.0f, 0.0f, NONE_IMPROVE);//-1.0 -0.008
//    
//    PID_struct_init(&gimbal.yaw_angle.pid, 300,5.0f, 0.0f, 0.0f, 0.0f, 0,0,0,NONE_IMPROVE);
//    PID_struct_init(&gimbal.yaw_spd.pid, 7, 1.0f, 0.0f,0.0f, 0.00, 0, 0.0f, 0.0f, NONE_IMPROVE);
//	
////    PID_struct_init(&gimbal.yaw_ecd.pid, 300,5.0f, 0.0f, 0.0f, 0.0f, 0,0,0,NONE_IMPROVE);
////	PID_struct_init(&gimbal.yaw_spd_ecd.pid, 7, 1.0f, 0.0f,0.0f, 0.00, 0, 0.0f, 0.0f, NONE_IMPROVE);
//	

//}

//float yaw_err;
//static void gimbal_pid_calc(void)
//{
//    float  pit_max, pit_min;
//    //位置环反馈 陀螺仪
//    //速度环反馈 陀螺仪
//    data_limit(gimbal.pit_angle.ref, pit_min, pit_max);
//    gimbal.pit_angle.fdb = imu_data.pitch;
//    gimbal.pit_spd.ref = pid_calc(&gimbal.pit_angle.pid, gimbal.pit_angle.ref, gimbal.pit_angle.fdb);
////    gimbal.pit_spd.fdb = -gimbal_imu.wy - arm_cos_f32(yaw_err) * chassis_imu.wy;
//    gimbal.pit_spd.fdb = imu_data.wy;
//    gimbal.pit_output = pid_calc(&gimbal.pit_spd.pid, gimbal.pit_spd.ref, gimbal.pit_spd.fdb);

//    //pid参数选择

//    
//    if (gimbal.yaw_angle.ref < 0) {
//        gimbal.yaw_angle.ref += 2 * PI;
//    } else if (gimbal.yaw_angle.ref > 2 * PI) {
//        gimbal.yaw_angle.ref -= 2 * PI;
//    }
//    //视觉测试
////    gimbal.yaw_angle.ref = FGT_agl_calc(&yaw_test);
//	
//    gimbal.yaw_angle.fdb = gimbal_imu.yaw;
//    //此yaw_err用于云台yaw环形控制
//    yaw_err = circle_error(gimbal.yaw_angle.ref, gimbal.yaw_angle.fdb, 360);
//    if (gimbal.start_up == 0 && yaw_err < 0.03f)//-------------------->无云台控制下注释
//        gimbal.start_up = 1;
//	
//    gimbal.yaw_spd.ref = pid_calc(&gimbal.yaw_angle.pid, gimbal.yaw_angle.fdb + yaw_err, gimbal.yaw_angle.fdb);    
////    gimbal.yaw_spd.fdb = gimbal_imu.wz + 0.4f * chassis_imu.wz;
//    gimbal.yaw_spd.fdb = gimbal_imu.wz;
//    gimbal.yaw_output = -pid_calc(&gimbal.yaw_spd.pid, gimbal.yaw_spd.ref, gimbal.yaw_spd.fdb);
////		if (gimbal.start_up == 0 && fabs(yaw_err)  > 0.03f)//-------------------->无云台控制下注释
////       gimbal.yaw_output  *= 0.1f;
//    
////    //无云台控制------->无陀螺仪, 通过ecd控制
//		if( rc_fsm_check(RC_LEFT_LD)  && 0 ){
//    gimbal.yaw_ecd.fdb = (float)yaw_motor.ecd / 8192 * 2  * PI;
//    yaw_err = circle_error(gimbal.yaw_ecd.ref, gimbal.yaw_ecd.fdb, 2*PI);
//    gimbal.yaw_spd_ecd.ref = pid_calc(&gimbal.yaw_ecd.pid, gimbal.yaw_ecd.fdb + yaw_err, gimbal.yaw_ecd.fdb);
//		
//    gimbal.yaw_spd_ecd.fdb = gimbal_imu.wz;
//    gimbal.yaw_output = pid_calc(&gimbal.yaw_spd_ecd.pid, gimbal.yaw_spd_ecd.ref, gimbal.yaw_spd_ecd.fdb);
//		
//		
//		}
//		
////		    if (gimbal.start_up == 0 && yaw_err < 0.03f) //无云台时起身判断
////        gimbal.start_up = 1;    

//}
