/*
  * @file gimbal_task.c
  * @version 1.0
  * @date  2025.11.19
	*
  * @brief 26������̨����
	*
  *	@author  myh
  *
  */
  
#include "gimbal_task.h"
#include "pid.h"
#include "math_calcu.h"
#include "remote_msg.h"
#include "shoot_task.h"
#include "mode_switch_task.h"
#include "control_def.h"

#include "can_comm.h"
#include "status_task.h"
#include "prot_T_imu.h"
#include "KalmanFilter.h"
#include "prot_vision.h"
#include "prot_rm_motor.h"
#include "prot_dm_motor.h"
#include "prot_dr16.h"
#include "prot_remote_fusion.h"
gimbal_t gimbal;

static void gimbal_init(void)
{
	scale.ch1 = RC_CH1_SCALE_IMU;//角度
	scale.ch2 = RC_CH2_SCALE_IMU;
    memset(&gimbal, 0, sizeof(gimbal_t));
   
//PID_struct_init(&gimbal.pit_angle.pid,  400,100.0f, 28.0f, 0.0f, 18.0f, 0,0,0,NONE_IMPROVE); //无参系统
//    PID_struct_init(&gimbal.pit_spd.pid, 3, 1, 0.0095,0.0, 0.0078, 0, 0.0f, 0.0f, NONE_IMPROVE); //无参系统
        
//    PID_struct_init(&gimbal.yaw_angle.pid, 10000,2000.0f, 0.0f, 0.0f, 0.0f, 0,0,0,NONE_IMPROVE);
//    PID_struct_init(&gimbal.yaw_spd.pid, 25000, 10000.0f, 0.0f,0.0f, 0.00, 0, 0.0f, 0.0f, NONE_IMPROVE);
	
	PID_struct_init(&gimbal.pit_angle.pid,  280,50.0f, 21.8f, 0.021f, 15.0f, 0,0,0,NONE_IMPROVE);  //有参系统
    PID_struct_init(&gimbal.pit_spd.pid, 3, 1, 0.021,0.0, 0.01, 0, 0.0f, 0.0f, NONE_IMPROVE); //有参系统
    
    PID_struct_init(&gimbal.yaw_angle.pid, 500,40.0f, 15.8f, 0.00f, 5.0f, 0,0,0,NONE_IMPROVE); //有参系统
    PID_struct_init(&gimbal.yaw_spd.pid, 25000, 10000.0f, 198.0f,0.0f, 13.00, 0, 0.0f, 0.0f, NONE_IMPROVE); //有参系统
	
//	PID_struct_init(&gimbal.yaw_angle.pid, 500,40.0f, 25.0f, 0.0f, 16.0f, 0,0,0,NONE_IMPROVE); //无参系统
//    PID_struct_init(&gimbal.yaw_spd.pid, 25000, 10000.0f, 150.0f,0.0f, 10.00, 0, 0.0f, 0.0f, NONE_IMPROVE); //无参系统
	
//  PID_struct_init(&gimbal.yaw_ecd.pid, 300,5.0f, 0.0f, 0.0f, 0.0f, 0,0,0,NONE_IMPROVE);
//	PID_struct_init(&gimbal.yaw_spd_ecd.pid, 7, 1.0f, 0.0f,0.0f, 0.00, 0, 0.0f, 0.0f, NONE_IMPROVE);
	
	gimbal.yaw_ff.K1 = 0.0f;
	gimbal.yaw_ff.K2 = 0.0f;
	gimbal.yaw_ff.OutMax = 25000;
}

float yaw_err;
float pit_err;
//float limit_err;
float vision_mpc_k = 1;
static void gimbal_pid_calc(void)
{
//	if (dm3507_motor.err_code != 1) {
//	dm_motor_init(&dm3507_motor,DM3507,PROT_CAN_CHANNEL_3,DM_3507_RECEIVE_ID,DM_3507_SEND_ID,DM_3507_RATIO,DM_3507_OFFEST);
////		return;
//    }
    //λ�û����� ������
    //�ٶȻ����� ������
    gimbal.pit_angle.ref = data_limit(gimbal.pit_angle.ref, 53, -8);
    gimbal.pit_angle.fdb = imu_data.pitch;
    gimbal.pit_spd.ref = pid_calc(&gimbal.pit_angle.pid, gimbal.pit_angle.fdb, gimbal.pit_angle.ref);
//	  gimbal.pit_spd.ref = -10;
	pit_err = fabsf(gimbal.pit_angle.ref  - gimbal.pit_angle.fdb);
//    gimbal.pit_spd.fdb = -gimbal_imu.wy - arm_cos_f32(yaw_err) * chassis_imu.wy;
    gimbal.pit_spd.fdb = imu_data.wy;
    gimbal.pit_output = pid_calc(&gimbal.pit_spd.pid, gimbal.pit_spd.fdb, gimbal.pit_spd.ref);
/*pitch���ʵ�ʿ��������*/
	if(ctrl_mode==PROTECT_MODE || dm3507_motor.err_code != 1)
	{
		dm3507_motor.t_ref = 0;
		dm_mit_control(&dm3507_motor);//����������������Э���еĿ��ƺ�������������can���������е��ã�����
	}
//	else if(dm3507_motor.t_rotor >=75)
//	{
//		dm3507_motor.t_ref = gimbal.pit_output * 0.7;
//		dm_mit_control(&dm3507_motor);//����������������Э���еĿ��ƺ�������������can���������е��ã�����
//	}
	else{
		if(gimbal.pit_output<0)
		{
			pitch_motor.t_ref = gimbal.pit_output*1.15;
		}
		else
		{
			pitch_motor.t_ref = gimbal.pit_output*0.75;
		}
		
		dm3507_motor.t_ref = gimbal.pit_output;
	//	dm3507_motor.t_ref = 0;
		dm_mit_control(&dm3507_motor);//����������������Э���еĿ��ƺ�������������can���������е��ã�����
	}
//--------------------yaw_pid_calu(imu)----------------------------------//
	/*yaw��λ*/
//	gimbal.gimbal_limit.yaw_angle_max = -imu_data.yaw - yaw_motor.ecd*MOTOR_ECD_TO_DEG  +0.0f;//
//	gimbal.gimbal_limit.yaw_angle_min = -imu_data.yaw - yaw_motor.ecd*MOTOR_ECD_TO_DEG  +0.0f;//
//	limit_err = 
	
    gimbal.yaw_angle.fdb = imu_data.yaw;
    //��yaw_err������̨yaw���ο���
    yaw_err = circle_error(gimbal.yaw_angle.ref, gimbal.yaw_angle.fdb, 360);
		
    gimbal.yaw_spd.ref = pid_calc(&gimbal.yaw_angle.pid, gimbal.yaw_angle.fdb, gimbal.yaw_angle.fdb + yaw_err) 
						+ vision_mpc_k * gimbal.yaw_vel_feedforward;

//	uint16_t ecd = yaw_motor.ecd;
										
//	if (ecd >= YAW_ECD_RIGHT) {
//        // ������λ����һ���������������ٶ�
//        float pull_back = -(float)(ecd - YAW_ECD_RIGHT) * YAW_LIMIT_KP;
//        // ֻ������pull_back�����صķ���
//        if (gimbal.yaw_spd.ref > pull_back) {
//            gimbal.yaw_spd.ref = pull_back;
//        }
//			gimbal.yaw_angle.ref = imu_data.yaw;  // ���� imu_data.yaw + ������Сƫ��
//    }
//    else if (ecd <= YAW_ECD_LEFT) {
//        // ������λ
//        float pull_back = (float)(YAW_ECD_LEFT - ecd) * YAW_LIMIT_KP;
//        // ֻ������pull_back�����صķ���
//        if (gimbal.yaw_spd.ref < pull_back) {
//            gimbal.yaw_spd.ref = pull_back;
//        }
//			gimbal.yaw_angle.ref = imu_data.yaw;  // ���� imu_data.yaw + ������Сƫ��

//    }
	
	gimbal.yaw_spd.fdb = imu_data.wz;
//	FeedForward_Calc(&gimbal.yaw_ff, gimbal.yaw_spd.ref);
	gimbal.yaw_output = -pid_calc(&gimbal.yaw_spd.pid, gimbal.yaw_spd.fdb, gimbal.yaw_spd.ref) - gimbal.yaw_ff.Out;
/*----------------------yaw_pid_calu(ecd)--------------*/

//	gimbal.yaw_ecd.fdb = (float)yaw_motor.ecd / 8192 * 2  * PI;
//	yaw_err = circle_error(gimbal.yaw_angle.ref, gimbal.yaw_ecd.fdb, 360);
//	gimbal.yaw_spd_ecd.ref = pid_calc(&gimbal.yaw_ecd.pid, gimbal.yaw_ecd.fdb + yaw_err, gimbal.yaw_ecd.fdb);
	
/*----------------------yaw���ʵ�ʿ������--------------*/
	if(ctrl_mode==PROTECT_MODE)
	{
		yaw_motor.tx_current = 0;
		dji_send_current(&yaw_motor, NULL, NULL, NULL);//����������������Э���еĿ��ƺ�������������can���������е��ã�����
	}
	else{
		yaw_motor.tx_current = gimbal.yaw_output;
		dji_send_current(&yaw_motor, NULL, NULL, NULL);//����������������Э���еĿ��ƺ�������������can���������е��ã�����
	}
/*--------------------------------------------------------*/
//�����д��imu����Ϊ0ʱ�����������Ƶļ���
}

void gimbal_PID_clear(gimbal_t *g)
{
    g->yaw_angle.ref = g->yaw_angle.fdb = imu_data.yaw;
    g->yaw_spd.ref   = g->yaw_spd.fdb = 0;
    g->pit_angle.ref = g->pit_angle.fdb = imu_data.pitch;
    g->pit_spd.ref   = g->pit_spd.fdb = 0;
    g->yaw_ecd.ref   = 0.0f;
    g->yaw_spd_ecd.ref = 0.0f;

	/* 2. 清除每个 PID 内部状态 */
    PID_clear(&g->yaw_angle.pid);
    PID_clear(&g->yaw_spd.pid);
    PID_clear(&g->pit_angle.pid);
    PID_clear(&g->pit_spd.pid);
    PID_clear(&g->yaw_ecd.pid);
    PID_clear(&g->yaw_spd_ecd.pid);
	dm_clear(&dm3507_motor);
    g->yaw_output = 0.0f;
    g->pit_output = 0.0f;
	g->yaw_ff.Last_DeltIn = 0.0f;
	g->yaw_ff.Now_DeltIn = 0.0f;
	g->yaw_ff.Out = 0.0f;
}

void gimbal_task(void const *argu)
{
	uint32_t mode_wake_time = osKernelSysTick();
    gimbal_init();
	for(;;)
	{
		mode_wake_time = osKernelSysTick();
        taskENTER_CRITICAL();
		switch( ctrl_mode )
		{
			case PROTECT_MODE:
			{
				gimbal_PID_clear(&gimbal);
				/*���dji�������ս�Ҫ���͵ĵ�������*/			
				yaw_motor.tx_current = 0;
				pit_motor.tx_current = 0;
			}break;
			
			case REMOTER_MODE:
			{
//				gimbal.yaw_ecd.ref   -= remote_fusion.channel.ch1 * scale.ecd_remote;
				gimbal.yaw_angle.ref += remote_fusion.channel.ch1 * (-1)*scale.ch1;
				gimbal.pit_angle.ref += remote_fusion.channel.ch2  * scale.ch2*(-1);
			}break;
			
			case KEYBOARD_MODE:
			{
				gimbal.pit_angle.ref   += (-remote_fusion.mouse.y) *  KEYBOARD_SCALE_PIT;
				gimbal.yaw_angle.ref   += (-remote_fusion.mouse.x) *  KEYBOARD_SCALE_YAW;
			}break;
			
			case VISION_MODE: /* �Ӿ�ģʽ */
			{
				vsn_gimbal_ref_calc();
			}break;
			default:break;
		}
		        taskEXIT_CRITICAL();

		gimbal_pid_calc();
//    	wdg_user_set_bit(WDG_BIT_TASK_GIMBAL);		
        osDelayUntil(&mode_wake_time, GIMBAL_PERIOD);
	}
}
