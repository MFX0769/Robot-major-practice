#include "debug_task.h"
#include "cmsis_os.h"
#include "usart.h"
#include "DataScope_DP.h"
#include "status_task.h"
#include "prot_rm_judge.h"
#include "data_log.h"
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
uint8_t debug_wave = 4; 

void log_scope_data_pkg(void)
{
    switch (debug_wave) {
        case 1: { /* 拨盘 */
//					log_scope_get_data(shoot.stir_mode);


 					log_scope_get_data(shoot.pid.trigger_ecd_ref);
					log_scope_get_data(shoot.pid.trigger_ecd_fdb);
//					log_scope_get_data(pid_trigger_ecd.pos_out);

//					log_scope_get_data(shoot.frequency_cnt);
//					log_scope_get_data(STIR_SERIES_SHOT_FLAG);
					log_scope_get_data(shoot.pid.trigger_spd_ref);
					log_scope_get_data(shoot.pid.trigger_spd_fdb);
//					log_scope_get_data(pid_trigger_spd.pos_out);
					
//					log_scope_get_data(trigger_motor.tx_current);
					log_scope_get_data(back_flag_tri);
					log_scope_get_data(back_flag_cnt);
					log_scope_get_data(back_flag);
				log_scope_get_data(err_time);

            break;
        }
        case 2: { /* 云台PIT PID IMU */
//					  log_scope_get_data(gimbal.gimbal_limit.pitch_angle_max);
//			      log_scope_get_data(gimbal.gimbal_limit.pitch_angle_min);
					
//						log_scope_get_data(vision.aim_status);
//						log_scope_get_data(vision .rx_freq);
						
//			          log_scope_get_data(imu_data.pitch);
					  log_scope_get_data(gimbal.pit_angle.ref);
            log_scope_get_data(gimbal.pit_angle.fdb);
				 
            log_scope_get_data(gimbal.pit_spd.ref);
            log_scope_get_data(gimbal.pit_spd.fdb);
////////					
//			log_scope_get_data(gimbal.pit_spd.pid.pout);		
//            log_scope_get_data(dm3507_motor.t_ref);
//            log_scope_get_data(dm3507_motor.t_set);
								  
//            log_scope_get_data(dm3507_motor.send_cnt);
//            log_scope_get_data(dm3507_motor.receive_cnt);
						

            break;
        }
        case 3: { /* 云台YAW PID IMU*/
////					 	log_scope_get_data(vision.target_yaw_angle);
////					 	log_scope_get_data(yaw_err);
////            log_scope_get_data(imu_data.yaw);
//			
//			
//			log_scope_get_data(gimbal.gimbal_limit.yaw_angle_max);
//            log_scope_get_data(gimbal.gimbal_limit.yaw_angle_min);
//					
            log_scope_get_data(gimbal.yaw_angle.ref);
            log_scope_get_data(gimbal.yaw_angle.fdb);
////            log_scope_get_data(moto_yaw.ecd);
//					
            
////            log_scope_get_data(gimbal.yaw_angle.pid.dout);
////		
			log_scope_get_data(yaw_motor.tx_current);
            log_scope_get_data(gimbal.yaw_spd.ref);
            log_scope_get_data(gimbal.yaw_spd.fdb);
//			log_scope_get_data(gimbal.yaw_spd.pid.pout);
			log_scope_get_data(vision.target_yaw_angle);
//            log_scope_get_data(yaw_motor.ecd);
////            log_scope_get_data(gimbal.yaw_spd.pid.pos_out);
////			log_scope_get_data(D_AGL_FeedForward_yaw.Out);

////            log_scope_get_data(imucnt);
//////					
////			log_scope_get_data(yaw_motor.send_cnt);
////			log_scope_get_data(yaw_can_receive_cnt);
////					
////					log_scope_get_data(gimbal.pit_angle.ref);
////					log_scope_get_data(gimbal.pit_angle.fdb);
////						
////					log_scope_get_data(gimbal.pit_spd.ref);
////					log_scope_get_data(gimbal.pit_spd.fdb);
////									
////					  log_scope_get_data(gimbal.gimbal_limit.yaw_angle_max);
////			      log_scope_get_data(gimbal.gimbal_limit.yaw_angle_min);
            break;
        }
		case 4: { 
					log_scope_get_data(vision.rx_cnt);
					log_scope_get_data(vision.target_pit_angle);
					log_scope_get_data(vision.target_yaw_angle);
			log_scope_get_data(gimbal.pit_angle.fdb);
            log_scope_get_data(gimbal.yaw_angle.fdb);
			log_scope_get_data(gimbal.yaw_spd.ref);
            log_scope_get_data(gimbal.yaw_spd.fdb);
            log_scope_get_data(yaw_motor.tx_current);

            break;
        }

        case 5: { /* 摩擦轮 */
 
					  log_scope_get_data(fric[0].spd_ref);
            log_scope_get_data(fric[1].spd_ref);
					
					  log_scope_get_data(fric_motor[0].speed_rpm);
            log_scope_get_data(fric_motor[1].speed_rpm);    
			log_scope_get_data(fric_can_recive_cnt);
            break;
				}
        case 6: { /* 陀螺仪 */
            log_scope_get_data(imu_data.pitch);
            log_scope_get_data(imu_data.yaw);
            log_scope_get_data(imu_data.wy);
            log_scope_get_data(imu_data.wz);
					
            log_scope_get_data(imu_data.wz);
			log_scope_get_data(imu_data.real_yaw);
            break;
        }
		case 7: { /* 热量测试相关 */
					
            log_scope_get_data(shoot.barrel.heat);   //当前枪管(理论)热量
            log_scope_get_data(shoot.barrel.heat_remain);   //当前枪管(理论)剩余热量
			log_scope_get_data(back_flag_tri);
			log_scope_get_data(back_flag_cnt);
			log_scope_get_data(back_flag);
			log_scope_get_data(err_time);
			log_scope_get_data(dm3507_motor.err_code);	
            break;
        }
//        case 8: { /*其余待打印*/
//		  log_scope_get_data(Shoot_Data.bullet_speed);//裁判系统回传射速   
//                log_scope_get_data(Shoot_Data.bullet_freq);
//			     log_scope_get_data(Shoot_Data.bullet_speed);
//			 log_scope_get_data(0.0);
//			log_scope_get_data(imu_data.pitch);
////			log_scope_get_data(-vision.pit_angle_error);
////			log_scope_get_data(-vision.yaw_angle_error);
//			log_scope_get_data(imu_data.yaw);
//			log_scope_get_data(gimbal.pid.yaw_angle_ref);
//			log_scope_get_data(gimbal.gimbal_limit.pitch_angle_max);
//			log_scope_get_data(gimbal.gimbal_limit.pitch_angle_min);
//					log_scope_get_data(moto_pit_4005.iq);
//					log_scope_get_data(moto_pit_4005.temperature);
//					            log_scope_get_data(moto_pit_4005.toq_i_ref);

////      log_scope_get_data(gimbal.pid.yaw_angle_ref);
//      log_scope_get_data(gimbal.pit_angle.ref);
//      log_scope_get_data(gimbal.pid.pit_angle_fdb);
//      log_scope_get_data(rc.ch2);
//      log_scope_get_data(ABS(gimbal.pid.yaw_angle_fdb - vsn_drive.target_yaw_angle));
//      log_scope_get_data(vsn_drive.yaw_min_err);
//      log_scope_get_data(vsn_drive.shoot_enable);//2025年4月23日 下午 这三行与视觉有关的get data，会导致视觉那边的帧率下降（从333下降到325 或 250）
//					
//					
//				log_scope_get_data(moto_pit_4005.total_encoder*360/655350);
//			log_scope_get_data(imu_data.pitch);
//			log_scope_get_data((float)Rad_conver_Angle(moto_pit.pos));
//			log_scope_get_data(0.0f);
//			log_scope_get_data(rc.ch1);
//			log_scope_get_data(rc.ch2);
//            break;
//        }
				
				       case 9: { /*imu*/
		  log_scope_get_data(imu_data.pitch);//裁判系统回传射速
			log_scope_get_data(imu_data.wy);//裁判系统回传射速
		  log_scope_get_data(imu_data.yaw);//裁判系统回传射速
		  log_scope_get_data(imu_data.wz);//裁判系统回传射速


            break;
        }
							 
		case 10: { /*imu*/
//		  log_scope_get_data(rc.ch1);
//		  log_scope_get_data(rc.ch2);
//		  log_scope_get_data(rc.ch3);
//		  log_scope_get_data(rc.ch4);
			log_scope_get_data(imu_data.pitch);
			log_scope_get_data(imu_data.yaw);
		  log_scope_get_data(yaw_err);
		  log_scope_get_data(gimbal.yaw_angle.ref);
		  log_scope_get_data(gimbal.yaw_angle.fdb);
//			 log_scope_get_data(remote_online.DT7_online_cnt);
//			 log_scope_get_data(remote_online.DT7_last_online_cnt);
//												 
//				log_scope_get_data(lock_flag);
//								log_scope_get_data(fric_can_receive_cnt);
//								log_scope_get_data(triger_can_receive_cnt);
            break;
        }
							 
				
        default: {
            break;
        }
    }
}

/* 串口上位机数据发送任务 */
void debug_task(void const* argu)
{
    uint32_t thread_wake_time = osKernelSysTick();
    for(;;)
    {
        thread_wake_time = osKernelSysTick();
//		log_printf("%.2f",imu_data.pitch);
      log_scope_data_output();		
            
        osDelayUntil(&thread_wake_time, 5);
    }
}
