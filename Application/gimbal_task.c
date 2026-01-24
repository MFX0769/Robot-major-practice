/*
  * @file gimbal_task.c
  * @version 1.0
  * @date  2025.11.19
	*
  * @brief 26赛季云台任务
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

gimbal_t gimbal;

static void gimbal_init(void)
{
	scale.ch1 = RC_CH1_SCALE_IMU;//灵敏度
	scale.ch2 = RC_CH2_SCALE_IMU;
    memset(&gimbal, 0, sizeof(gimbal_t));
   
//	PID_struct_init(&gimbal.pit_angle.pid,  200,5.0f, 7.0f, 0.0f, 0.0f, 0,0,0,NONE_IMPROVE);
//    PID_struct_init(&gimbal.pit_spd.pid, 3, 1.0, 0.005f,0.0001f, 0.00, 0, 0.0f, 0.0f, NONE_IMPROVE);
//    
//    PID_struct_init(&gimbal.yaw_angle.pid, 10000,2000.0f, 15.0f, 0.0f, 3.0f, 0,0,0,NONE_IMPROVE);
//    PID_struct_init(&gimbal.yaw_spd.pid, 25000, 10000.0f, 80.0f,1.0f, 0.00, 0, 0.0f, 0.0f, NONE_IMPROVE);
	
	PID_struct_init(&gimbal.pit_angle.pid,  200,5.0f, 0.0f, 0.0f, 0.0f, 0,0,0,NONE_IMPROVE);
    PID_struct_init(&gimbal.pit_spd.pid, 3, 1.0, 0.0,0.0, 0.00, 0, 0.0f, 0.0f, NONE_IMPROVE);
    
    PID_struct_init(&gimbal.yaw_angle.pid, 10000,2000.0f, 0.0f, 0.0f, 0.0f, 0,0,0,NONE_IMPROVE);
    PID_struct_init(&gimbal.yaw_spd.pid, 25000, 10000.0f, 0.0f,0.0f, 0.00, 0, 0.0f, 0.0f, NONE_IMPROVE);
	
//  PID_struct_init(&gimbal.yaw_ecd.pid, 300,5.0f, 0.0f, 0.0f, 0.0f, 0,0,0,NONE_IMPROVE);
//	PID_struct_init(&gimbal.yaw_spd_ecd.pid, 7, 1.0f, 0.0f,0.0f, 0.00, 0, 0.0f, 0.0f, NONE_IMPROVE);
	

}

float yaw_err;
static void gimbal_pid_calc(void)
{
//	if(ctrl_mode == PROTECT_MODE){return;}
    //位置环反馈 陀螺仪
    //速度环反馈 陀螺仪
    gimbal.pit_angle.ref = data_limit(gimbal.pit_angle.ref, 50, -12);
    gimbal.pit_angle.fdb = imu_data.pitch;
    gimbal.pit_spd.ref = pid_calc(&gimbal.pit_angle.pid, gimbal.pit_angle.fdb, gimbal.pit_angle.ref);
//    gimbal.pit_spd.fdb = -gimbal_imu.wy - arm_cos_f32(yaw_err) * chassis_imu.wy;
    gimbal.pit_spd.fdb = imu_data.wy;
    gimbal.pit_output = pid_calc(&gimbal.pit_spd.pid, gimbal.pit_spd.fdb, gimbal.pit_spd.ref);
/*pitch电机实际控制输出名*/
	if(ctrl_mode==PROTECT_MODE)
	{
		dm3507_motor.t_ref = 0;
		dm_mit_control(&dm3507_motor);//！！！新增：调用协议中的控制函数，而不是在can发送任务中调用！！！
	}
	else{
		pitch_motor.t_ref = gimbal.pit_output;
		dm3507_motor.t_ref = gimbal.pit_output;
	//	dm3507_motor.t_ref = 0;
		dm_mit_control(&dm3507_motor);//！！！新增：调用协议中的控制函数，而不是在can发送任务中调用！！！
	}
//--------------------yaw_pid_calu----------------------------------//
	/*yaw限位*/
	gimbal.gimbal_limit.yaw_angle_max = imu_data.yaw + yaw_motor.ecd*MOTOR_ECD_TO_DEG  -0.0f;//
	gimbal.gimbal_limit.yaw_angle_min = imu_data.yaw + yaw_motor.ecd*MOTOR_ECD_TO_DEG  -0.0f;//
	
	
	gimbal.yaw_angle.ref = data_limit(gimbal.yaw_angle.ref ,gimbal.gimbal_limit.yaw_angle_max,gimbal.gimbal_limit.yaw_angle_min);	//目标值限幅	
    gimbal.yaw_angle.fdb = imu_data.yaw;
	
	
    //此yaw_err用于云台yaw环形控制
    yaw_err = circle_error(gimbal.yaw_angle.ref, gimbal.yaw_angle.fdb, 360);
	
    gimbal.yaw_spd.ref = pid_calc(&gimbal.yaw_angle.pid, gimbal.yaw_angle.fdb, gimbal.yaw_angle.fdb + yaw_err);    
	gimbal.yaw_spd.fdb = imu_data.wz;
	gimbal.yaw_output = -pid_calc(&gimbal.yaw_spd.pid, gimbal.yaw_spd.fdb, gimbal.yaw_spd.ref);
/*----------------------yaw电机实际控制输出--------------*/
	if(ctrl_mode==PROTECT_MODE)
	{
		yaw_motor.tx_current = 0;
		dji_send_current(&yaw_motor, NULL, NULL, NULL);//！！！新增：调用协议中的控制函数，而不是在can发送任务中调用！！！
	}
	else{
		yaw_motor.tx_current = gimbal.yaw_output;
		dji_send_current(&yaw_motor, NULL, NULL, NULL);//！！！新增：调用协议中的控制函数，而不是在can发送任务中调用！！！
	}
/*--------------------------------------------------------*/
//下面可写当imu意外为0时靠编码器控制的计算
}

void gimbal_PID_clear(gimbal_t *g)
{
    g->yaw_angle.ref = g->yaw_angle.fdb = imu_data.yaw;
    g->yaw_spd.ref   = g->yaw_spd.fdb = 0;
    g->pit_angle.ref = g->pit_angle.fdb = imu_data.pitch;
    g->pit_spd.ref   = g->pit_spd.fdb = 0;
    g->yaw_ecd.ref   = 0.0f;
    g->yaw_spd_ecd.ref = 0.0f;

	/* 2. 把每个 PID 内部状态也清零 */
    PID_clear(&g->yaw_angle.pid);
    PID_clear(&g->yaw_spd.pid);
    PID_clear(&g->pit_angle.pid);
    PID_clear(&g->pit_spd.pid);
    PID_clear(&g->yaw_ecd.pid);
    PID_clear(&g->yaw_spd_ecd.pid);
	dm_clear(&dm3507_motor);
    g->yaw_output = 0.0f;
    g->pit_output = 0.0f;
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
				/*针对dji电机，清空将要发送的电流数据*/			
				yaw_motor.tx_current = 0;
				pit_motor.tx_current = 0;
			}break;
			
			case REMOTER_MODE:
			{
				gimbal.yaw_angle.ref += rc.ch1 * (-1)*scale.ch1;
				gimbal.pit_angle.ref += rc.ch2 * scale.ch2*(-1);
			}break;
			
			case KEYBOARD_MODE:
			{
				gimbal.pit_angle.ref   += (-rc.mouse.y) *  KEYBOARD_SCALE_PIT;
				gimbal.yaw_angle.ref   += (-rc.mouse.x) *  KEYBOARD_SCALE_YAW;
			}break;
			
			case VISION_MODE: /* 视觉模式 */
			{
				vsn_gimbal_ref_calc();
			}break;
			default:break;
		}
		gimbal_pid_calc();
//    	wdg_user_set_bit(WDG_BIT_TASK_GIMBAL);		
        taskEXIT_CRITICAL();
        osDelayUntil(&mode_wake_time, GIMBAL_PERIOD);
	}
}
