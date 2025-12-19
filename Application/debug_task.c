#include "debug_task.h"
#include "cmsis_os.h"
#include "usart.h"
#include "DataScope_DP.h"
#include "status_task.h"
#include "prot_rm_judge.h"

/* 待添加头文件 */
#include "math.h"
#include "pid.h"
#include "prot_T_imu.h"
#include "gimbal_task.h"
#include "prot_vision.h"
#include "prot_vision.h"

#include "prot_TriggerMotor.h"
#include "shoot_task.h"
#include "bsp_can.h"
#include "pid.h"
#include "gimbal_task.h"
#include "status_task.h"
#include "remote_msg.h"
#include "mode_switch_task.h"
#include "control_def.h"
#include "shoot_task.h"
#include "string.h"
#include "remote_msg.h"
#include "prot_dm_motor.h"
#include "prot_rm_motor.h"

//#define DEBUG_TASK_ENABLE

//#define ABS(x) ((x>0)? (x): (-(x)))

//待打印变量
extern pid_t pid_trigger_ecd;
extern pid_t pid_trigger_spd;

extern float* last_imu_yaw_data;
extern float std;
	
float float_test;
uint32_t debug_fcnt=0;
/* 用以选择打印的数据类型 */
uint8_t debug_wave = 0; 

/* --------------------------------------------------------------------------------------------------
视觉调试步骤与涉及变量
DataScope_Get_Channel_Data();
0. 查看数据方差（p,y,d,t,pos）    vsn_std_switch, std
1. 角度幅值匹配                   视觉侧的Z偏移量
2. 角度相位匹配                   match_time_angle
3. 角速度幅值匹配                 kal_R_angle, kal_R_yaw_aim_spd, kal_R_yaw_imu_spd
   角速度相位匹配                 match_time_omega
4. 延时测试                    

offset_time
5. 距离与飞行时间滤波与相位匹配   kal_R_dis_tof, match_time_dis_tof
6. 视觉滤波器收敛时间             stable_time_flt
-------------------------------------------------------------------------------------------------- */

/* 其他待打印数据采样 */
static void OtherDataWavePkg(uint8_t debug_wave) {
    switch (debug_wave) {
        case 110: { /* 拨盘 */
						DataScope_Get_Channel_Data(shoot.stir_mode);


 					  DataScope_Get_Channel_Data(shoot.pid.trigger_ecd_ref);
            DataScope_Get_Channel_Data(shoot.pid.trigger_ecd_fdb);
					
						DataScope_Get_Channel_Data(shoot.frequency_cnt);
						DataScope_Get_Channel_Data(STIR_SERIES_SHOT_FLAG);
					  DataScope_Get_Channel_Data(shoot.pid.trigger_spd_ref);
            DataScope_Get_Channel_Data(shoot.pid.trigger_spd_fdb);
            DataScope_Get_Channel_Data(pid_trigger_spd.pos_out);
					
					DataScope_Get_Channel_Data(trigger_can_receive_cnt);
					

            break;
        }
        case 111: { /* 云台PIT PID IMU */
//					  DataScope_Get_Channel_Data(gimbal.gimbal_limit.pitch_angle_max);
//			      DataScope_Get_Channel_Data(gimbal.gimbal_limit.pitch_angle_min);
					
//						DataScope_Get_Channel_Data(vision.aim_status);
//						DataScope_Get_Channel_Data(vision .rx_freq);
//						DataScope_Get_Channel_Data(vision.rx_cnt);
//						DataScope_Get_Channel_Data(vision.target_pit_angle);
					
//			          DataScope_Get_Channel_Data(imu_data.pitch);
					  DataScope_Get_Channel_Data(gimbal.pit_angle.ref);
            DataScope_Get_Channel_Data(gimbal.pit_angle.fdb);
				 
            DataScope_Get_Channel_Data(gimbal.pit_spd.ref);
            DataScope_Get_Channel_Data(gimbal.pit_spd.fdb);
//////					
					
            DataScope_Get_Channel_Data(dm3507_motor.t_ref);
            DataScope_Get_Channel_Data(dm3507_motor.t_set);
								  
//            DataScope_Get_Channel_Data(dm3507_motor.send_cnt);
//            DataScope_Get_Channel_Data(dm3507_motor.receive_cnt);
						
					
//            DataScope_Get_Channel_Data(moto_pit_4005.ecd_pos);
//            DataScope_Get_Channel_Data(moto_pit_4005.raw_encoder);
//            DataScope_Get_Channel_Data(moto_pit_4005.round_cnt);
//            DataScope_Get_Channel_Data(moto_pit_4005.speed);

//					
//            DataScope_Get_Channel_Data(moto_pit_4005.vel_ref);
//            DataScope_Get_Channel_Data(imu_data.wy);
//            DataScope_Get_Channel_Data(moto_pit_4005.toq_i_ref);

            break;
        }
        case 112: { /* 云台YAW PID IMU*/
//					 	DataScope_Get_Channel_Data(vision.target_yaw_angle);
//					 	DataScope_Get_Channel_Data(yaw_err);
//            DataScope_Get_Channel_Data(imu_data.yaw);
//					  DataScope_Get_Channel_Data(gimbal.gimbal_limit.yaw_angle_max);
//            DataScope_Get_Channel_Data(gimbal.gimbal_limit.yaw_angle_min);
					
            DataScope_Get_Channel_Data(gimbal.yaw_angle.ref);
            DataScope_Get_Channel_Data(gimbal.yaw_angle.fdb);
//            DataScope_Get_Channel_Data(moto_yaw.ecd);
					
//            DataScope_Get_Channel_Data(gimbal.yaw_angle.pid.pout);
//            DataScope_Get_Channel_Data(gimbal.yaw_angle.pid.dout);
//		

            DataScope_Get_Channel_Data(gimbal.yaw_spd.ref);
            DataScope_Get_Channel_Data(gimbal.yaw_spd.fdb);
//            DataScope_Get_Channel_Data(gimbal.yaw_spd.pid.pos_out);
//			DataScope_Get_Channel_Data(D_AGL_FeedForward_yaw.Out);

//            DataScope_Get_Channel_Data(imucnt);
//					
//					DataScope_Get_Channel_Data(yaw_motor.send_cnt);
//					DataScope_Get_Channel_Data(yaw_can_receive_cnt);
//									
//					  DataScope_Get_Channel_Data(gimbal.gimbal_limit.yaw_angle_max);
//			      DataScope_Get_Channel_Data(gimbal.gimbal_limit.yaw_angle_min);
            break;
        }
				case 118: { /* 云台ROLL roll IMU*/
//            DataScope_Get_Channel_Data(gimbal.pid.roll_angle_ref);
//            DataScope_Get_Channel_Data(gimbal.pid.roll_angle_fdb);
//            DataScope_Get_Channel_Data(gimbal.pid.roll_spd_ref);
//            DataScope_Get_Channel_Data(gimbal.pid.roll_spd_fdb);
            break;
        }
//        case 113: { /* 云台YAW PID ECD*/
//            DataScope_Get_Channel_Data(gimbal.pid.yaw_angle_ref);
//            DataScope_Get_Channel_Data(moto_yaw.ecd);
//            DataScope_Get_Channel_Data(pid_yaw_ecd.pos_out);
//            DataScope_Get_Channel_Data(moto_yaw.speed_rpm);
//            break;
//        }
//        case 114: {/* 云台PIT PID ECD*/
//            DataScope_Get_Channel_Data(gimbal.pid.pit_ecd_ref);
//            DataScope_Get_Channel_Data(gimbal.pid.pit_ecd_fdb);
//					
//            DataScope_Get_Channel_Data(gimbal.pid.pit_spd_ref);
//            DataScope_Get_Channel_Data(dm4310_vel_lpf);

//            DataScope_Get_Channel_Data(motor_dm4310_pitch.toq_ref);
////            DataScope_Get_Channel_Data(motor_dm4310_pitch.toq);
//            break;
//        }
        case 115: { /* 摩擦轮 */
 
					  DataScope_Get_Channel_Data(fric[0].spd_ref);
            DataScope_Get_Channel_Data(fric[1].spd_ref);
					
					  DataScope_Get_Channel_Data(fric_motor[0].speed_rpm);
            DataScope_Get_Channel_Data(fric_motor[1].speed_rpm);    
			DataScope_Get_Channel_Data(fric_can_recive_cnt);

				}
            break;
        case 116: { /* 陀螺仪 */
            DataScope_Get_Channel_Data(imu_data.pitch);
            DataScope_Get_Channel_Data(imu_data.yaw);
            DataScope_Get_Channel_Data(imu_data.wy);
            DataScope_Get_Channel_Data(imu_data.wz);
					
            DataScope_Get_Channel_Data(imu_data.wz);
			DataScope_Get_Channel_Data(imu_data.real_yaw);
            break;
        }
//				case 117: { /* 力矩测试相关 */
//					
//					//ROLL
//            DataScope_Get_Channel_Data(motor_dm4310_roll.avrg_toq);
//            DataScope_Get_Channel_Data(motor_dm4310_roll.max_toq);
//					
//					  DataScope_Get_Channel_Data(motor_dm4310_roll.toq);
//            DataScope_Get_Channel_Data(gimbal.pid.roll_angle_fdb);
//					
//					//PITCH
//	          DataScope_Get_Channel_Data(moto_pit_4005.avrg_toq);
//            DataScope_Get_Channel_Data(moto_pit_4005.max_toq);
//					
//					  DataScope_Get_Channel_Data(moto_pit_4005.toq_cal);
//            DataScope_Get_Channel_Data(gimbal.pid.pit_angle_fdb);				
//					
//            break;
//        }
//        case 124: { /*其余待打印*/
//		  DataScope_Get_Channel_Data(Shoot_Data.bullet_speed);//裁判系统回传射速   
//                DataScope_Get_Channel_Data(Shoot_Data.bullet_freq);
//			     DataScope_Get_Channel_Data(Shoot_Data.bullet_speed);
//			 DataScope_Get_Channel_Data(0.0);
//			DataScope_Get_Channel_Data(imu_data.pitch);
////			DataScope_Get_Channel_Data(-vision.pit_angle_error);
////			DataScope_Get_Channel_Data(-vision.yaw_angle_error);
//			DataScope_Get_Channel_Data(imu_data.yaw);
//			DataScope_Get_Channel_Data(gimbal.pid.yaw_angle_ref);
//			DataScope_Get_Channel_Data(gimbal.gimbal_limit.pitch_angle_max);
//			DataScope_Get_Channel_Data(gimbal.gimbal_limit.pitch_angle_min);
//					DataScope_Get_Channel_Data(moto_pit_4005.iq);
//					DataScope_Get_Channel_Data(moto_pit_4005.temperature);
//					            DataScope_Get_Channel_Data(moto_pit_4005.toq_i_ref);

////      DataScope_Get_Channel_Data(gimbal.pid.yaw_angle_ref);
//      DataScope_Get_Channel_Data(gimbal.pit_angle.ref);
//      DataScope_Get_Channel_Data(gimbal.pid.pit_angle_fdb);
//      DataScope_Get_Channel_Data(rc.ch2);
//      DataScope_Get_Channel_Data(ABS(gimbal.pid.yaw_angle_fdb - vsn_drive.target_yaw_angle));
//      DataScope_Get_Channel_Data(vsn_drive.yaw_min_err);
//      DataScope_Get_Channel_Data(vsn_drive.shoot_enable);//2025年4月23日 下午 这三行与视觉有关的get data，会导致视觉那边的帧率下降（从333下降到325 或 250）
//					
//					
//				DataScope_Get_Channel_Data(moto_pit_4005.total_encoder*360/655350);
//			DataScope_Get_Channel_Data(imu_data.pitch);
//			DataScope_Get_Channel_Data((float)Rad_conver_Angle(moto_pit.pos));
//			DataScope_Get_Channel_Data(0.0f);
//			DataScope_Get_Channel_Data(rc.ch1);
//			DataScope_Get_Channel_Data(rc.ch2);
//            break;
//        }
				
				       case 125: { /*imu*/
		  DataScope_Get_Channel_Data(imu_data.pitch);//裁判系统回传射速
			DataScope_Get_Channel_Data(imu_data.wy);//裁判系统回传射速
		  DataScope_Get_Channel_Data(imu_data.yaw);//裁判系统回传射速
		  DataScope_Get_Channel_Data(imu_data.wz);//裁判系统回传射速


            break;
        }
							 
//		case 126: { /*imu*/
//		  DataScope_Get_Channel_Data(rc.ch1);
//		  DataScope_Get_Channel_Data(rc.ch2);
//		  DataScope_Get_Channel_Data(rc.ch3);
//		  DataScope_Get_Channel_Data(rc.ch4);


//			 DataScope_Get_Channel_Data(remote_online.DT7_online_cnt);
//			 DataScope_Get_Channel_Data(remote_online.DT7_last_online_cnt);
//												 
//				DataScope_Get_Channel_Data(lock_flag);
//								DataScope_Get_Channel_Data(fric_can_receive_cnt);
//								DataScope_Get_Channel_Data(triger_can_receive_cnt);
//            break;
//        }
							 
				
        default: {
            break;
        }
    }
}

void DataWavePkg(void) {
//    VisionDataWavePkg(111);
//    OtherDataWavePkg(110); //Trigger
//      OtherDataWavePkg(111); //gimbal_pit_imu
		  OtherDataWavePkg(112); //gimbal_yaw_imu
//		OtherDataWavePkg(118); //gimbal_roll_imu
//		OtherDataWavePkg(113); //yaw_ecd
//		OtherDataWavePkg(114); //pit_ecd
//		OtherDataWavePkg(115); //Fric
//		OtherDataWavePkg(116); //T_IMU
//  		OtherDataWavePkg(117); //能量相关
//	    OtherDataWavePkg(124); //其余待打印
//	    OtherDataWavePkg(126); //rc
//    OtherDataWavePkg(debug_wave);
}

/* 串口上位机数据发送任务 */
void debug_task(void const *argu) {
    uint32_t thread_wake_time = osKernelSysTick();
    for (;;) {
//        wdg_user_set_bit(WDG_BIT_TASK_DEBUG);
//#ifdef DEBUG_TASK_ENABLE
        taskENTER_CRITICAL();
        DataWave(&huart6);
			   debug_fcnt++;
		 
        taskEXIT_CRITICAL();
        osDelayUntil(&thread_wake_time, 5);
//#elseif
//      osDelay(100);  
//#endif
        
    }
}

