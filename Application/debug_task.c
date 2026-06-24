#include "debug_task.h"
#include "config.h"
#include "cmsis_os.h"
#include "usart.h"
#include "DataScope_DP.h"
#include "status_task.h"
#include "prot_rm_judge.h"
#include "data_log.h"
/* ������ͷ�ļ� */
#include "math.h"
#include "pid.h"
#include "prot_T_imu.h"
#include "gimbal_task.h"
#include "prot_vision.h"
#include "prot_vision.h"

#include "prot_TriggerMotor.h"
#include "prot_8irtraking.h"
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
#include "prot_remote_fusion.h"
#include "prot_sbus.h"
//#define DEBUG_TASK_ENABLE

//#define ABS(x) ((x>0)? (x): (-(x)))

//����ӡ����
extern pid_t pid_trigger_ecd;
extern pid_t pid_trigger_spd;

extern float* last_imu_yaw_data;
extern float std;
	
float float_test;
extern dji_motor_t chassis_2006[4];
/* ����ѡ���ӡ���������� */
uint8_t debug_wave = 13;

void log_scope_data_pkg(void)
{
    switch (debug_wave) {
        case 1: { /* ���� */
//					log_scope_get_data(shoot.stir_mode);


 					log_scope_get_data(shoot.pid.trigger_ecd_ref);
					log_scope_get_data(shoot.pid.trigger_ecd_fdb);
//					log_scope_get_data(pid_trigger_ecd.pos_out);

//					log_scope_get_data(shoot.frequency_cnt);
					log_scope_get_data(trigger_motor.receive_cnt);
					log_scope_get_data(shoot.pid.trigger_spd_ref);
					log_scope_get_data(shoot.pid.trigger_spd_fdb);
					log_scope_get_data(pid_trigger_spd.pos_out);
					
					log_scope_get_data(trigger_motor.tx_current);
//					log_scope_get_data(back_flag_tri);
//					log_scope_get_data(back_flag_cnt);
//					log_scope_get_data(back_flag);
//				log_scope_get_data(err_time);

            break;
        }
        case 2: { /* ��̨PIT PID IMU */

						
//			          log_scope_get_data(imu_data.pitch);
					  log_scope_get_data(gimbal.pit_angle.ref);
            log_scope_get_data(gimbal.pit_angle.fdb);
				 
            log_scope_get_data(gimbal.pit_spd.ref);
            log_scope_get_data(gimbal.pit_spd.fdb);
////////					
			log_scope_get_data(dm3507_motor.t_rotor);		
            log_scope_get_data(dm3507_motor.t_ref);
            log_scope_get_data(dm3507_motor.err_code);
								  
            log_scope_get_data(dm3507_motor.send_cnt);
            log_scope_get_data(dm3507_motor.receive_cnt);
						

            break;
        }
        case 3: { /* ��̨YAW PID IMU*/
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
//			log_scope_get_data(yaw_motor.tx_current);
            log_scope_get_data(gimbal.yaw_spd.ref);
            log_scope_get_data(gimbal.yaw_spd.fdb);
//			log_scope_get_data(gimbal.yaw_spd.pid.pout);
//			log_scope_get_data(vision.target_yaw_angle);
//			log_scope_get_data(dm3507_motor.receive_cnt);
//			log_scope_get_data(dm3507_motor.err_code);
//								  
//            log_scope_get_data(gimbal.pit_angle.fdb);
//			log_scope_get_data(dm3507_motor.t_rotor);		
//            log_scope_get_data(dm3507_motor.t_ref);
						
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
//					log_scope_get_data(vision.target_pit_angle);
						log_scope_get_data(gimbal.pit_angle.ref);
			log_scope_get_data(gimbal.pit_angle.fdb);
			log_scope_get_data(gimbal.pit_spd.ref);
            log_scope_get_data(gimbal.pit_spd.fdb);
//			log_scope_get_data(vision.target_yaw_angle);
			            log_scope_get_data(gimbal.yaw_angle.ref);

            log_scope_get_data(gimbal.yaw_angle.fdb);

						log_scope_get_data(dm3507_motor.t_rotor);		
					log_scope_get_data(vision.rx[0].data.fire);
			log_scope_get_data(vision.shoot_enable);
            break;
        }

        case 5: { /* Ħ���� */
 
			log_scope_get_data(fric[0].spd_ref);
            log_scope_get_data(fric[1].spd_ref);
					
			log_scope_get_data(fric_motor[0].speed_rpm);
            log_scope_get_data(fric_motor[1].speed_rpm);    
			log_scope_get_data(fric_can_recive_cnt);
            break;
				}
        case 6: { /* ������ */
            log_scope_get_data(imu_data.pitch);
            log_scope_get_data(imu_data.yaw);
            log_scope_get_data(imu_data.wy);
            log_scope_get_data(imu_data.wz);
					
            log_scope_get_data(imu_data.wz);
			log_scope_get_data(imu_data.real_yaw);
            break;
        }
		case 7: { /* ����������� */
					
            log_scope_get_data(shoot.barrel.heat);   //��ǰǹ��(����)����
            log_scope_get_data(shoot.barrel.heat_remain);   //��ǰǹ��(����)ʣ������
			log_scope_get_data(back_flag_tri);
			log_scope_get_data(back_flag_cnt);
			log_scope_get_data(back_flag);
			log_scope_get_data(err_time);
			log_scope_get_data(dm3507_motor.err_code);	
            break;
        }
//        case 8: { /*�������ӡ*/
//		  log_scope_get_data(Shoot_Data.bullet_speed);//����ϵͳ�ش�����   
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
//      log_scope_get_data(vsn_drive.shoot_enable);//2025��4��23�� ���� ���������Ӿ��йص�get data���ᵼ���Ӿ��Ǳߵ�֡���½�����333�½���325 �� 250��
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
		  log_scope_get_data(remote_fusion.channel.ch1);//����ϵͳ�ش�����
		  log_scope_get_data(vtm_rc.cns);//����ϵͳ�ش�����
		log_scope_get_data(dm3507_motor.receive_cnt);
	         log_scope_get_data(dm3507_motor.err_code);
			log_scope_get_data(dr16_online_cnt);
		log_scope_get_data(imu_data.pitch);

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
							 
				
        case 11: { /* 底盘电机ecd + SBUS四通道，参考D:\侧case11(电机)并加上遥控器 */
            log_scope_get_data(chassis_2006[0].ecd);
            log_scope_get_data(chassis_2006[1].ecd);
            log_scope_get_data(chassis_2006[2].ecd);
            log_scope_get_data(chassis_2006[3].ecd);
            log_scope_get_data(wbus_rc.data.ch1);
            log_scope_get_data(wbus_rc.data.ch2);
            log_scope_get_data(wbus_rc.data.ch3);
            log_scope_get_data(wbus_rc.data.ch4);
            break;
        }
        case 12: { /* 专看SBUS遥控器通道 */
            log_scope_get_data(wbus_rc.data.ch1);
            log_scope_get_data(wbus_rc.data.ch2);
            log_scope_get_data(wbus_rc.data.ch3);
            log_scope_get_data(wbus_rc.data.ch4);
            log_scope_get_data(wbus_rc.data.ch5);
            log_scope_get_data(wbus_rc.data.ch6);
            log_scope_get_data(wbus_rc.data.ch7);
            log_scope_get_data(wbus_rc.data.ch8);
            log_scope_get_data(wbus_rc.data.sw1);
            log_scope_get_data(wbus_rx_cnt);
            log_scope_get_data(wbus_rx_freq);
            break;
        }
        case 13: { /* 巡线模块算出来的线位置 */
            const prot_8irtraking_data_t *line_data = prot_8irtraking_get_data();
            log_scope_get_data(line_data->state);
            log_scope_get_data(line_data->position);
            break;
        }
        default: {
            break;
        }
    }
}

/* ������λ�����ݷ������� */
void debug_task(void const* argu)
{
    uint32_t thread_wake_time = osKernelSysTick();
    for(;;)
    {
        thread_wake_time = osKernelSysTick();
#if DEBUG_VOFA_ENABLE
        log_scope_data_output();
#endif
            
        osDelayUntil(&thread_wake_time, 5);
    }
}
