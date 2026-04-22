/**
  * @file bsp_FricMotor.c
  * @version 1.0
  * @date  2022.3.14
	*
  * @brief 3508发射器代码
	*
  *	@author  22空中组
  *
  */

#include "prot_FricMotor.h"
#include "pid.h"
#include "math_calcu.h"
#include "remote_msg.h"
#include "shoot_task.h"
#include "mode_switch_task.h"
#include "control_def.h"
#include "can_comm.h"
#include "prot_rm_motor.h"

extern TaskHandle_t can_msg_send_task_t;
ramp_function_source_t Fric_speed;

/*********************************************************************/
pid_t fric_spd_pid[2];
fric_pid_t fric[2];
/*********************************************************************/
//Kalman1_param_t err_Kalman[2];                  //一维卡尔曼滤波结构体
/**
  * @name    	  FricMotor_init
	* @brief 			摩擦轮PID初始化
  * @param 			None
	* @note   		连发20 2 20  单发12 0.05 20.0
	* @imporvement[now] None
  */
	
void FricMotor_init(void)
{	

					
     PID_struct_init(&fric_spd_pid[FRIC_LEFT], 16000, 0,
                    5, 0, 0,0,0,0,Integral_Limit);
    PID_struct_init(&fric_spd_pid[FRIC_RIGHT], 16000, 0,
                    5, 0, 0,0,0,0,Integral_Limit);
									
    shoot.firc_mode = FIRC_MODE_STOP;
	shoot.stir_mode = STIR_MODE_PROTECT;
//    shoot.stir_mode = STIR_MODE_PROTECT;

    Fric_speed.input = 0;
    Fric_speed.out  = 0;
    Fric_speed.frame_period = 0.01;

    /* 初始化射速 */
    shoot.shoot_speed = 28;  
	

}

/**
  * @name     FricMotor_Control
  * @brief    摩擦轮控制
  * @param    None
  * @retval   None
  * @note     下面是数据是没有拨盘给初速的，只是在没有裁判系统的时候粗略的测的，所以，加上拨盘之后实际射速会比这个高1m/s 
								//4900  17.3-18.4
								//4800  16.7-17.5
								//4500 14.5-15.3
								//7400  27.8-29.4
								//7600 28.5-29.5 //试过掉到26
  */


void FricMotor_Control(void)
{        
	static int a,b=0;
	
    switch( shoot.firc_mode )
    {
        case FIRC_MODE_STOP:
        {
					
					/*斜坡函数 慢慢停，防止反电动势过大烧坏其他器件*/
			ramp_calc(&Fric_speed,-2.0f,1.0f,HIGH_SPEED,0);
//		   if(lock_flag==1)
//           {
//				ramp_calc(&Fric_speed,-2.0f,1.0f,HIGH_SPEED,0);

//		   }
//			else if(lock_flag==2)//外八解锁的话，就摩擦轮慢慢转
//            {
//				ramp_calc(&Fric_speed,-2.0f,1.0f,3000,0);

//			}
						
//						/*左右摩擦轮电控改方向*/
//					fric[0].spd_ref = -Fric_speed.out;
//					fric[1].spd_ref = Fric_speed.out;	
					  
					  fric[0].spd_ref = Fric_speed.out;
					  fric[1].spd_ref = -Fric_speed.out;
							
						/*速度赋值与PID计算*/
						for(b=0;b<2;b++)
						{
							fric[b].spd_fdb = fric_motor[b].speed_rpm;
							fric[b].current = pid_calc(&fric_spd_pid[b],fric[b].spd_fdb ,fric[b].spd_ref );
							fric_motor[b].tx_current = fric[b].current;
							if( ABS(fric[b].spd_fdb) < 500)//停下的时候不给电
								fric_motor[b].tx_current = 0 ;
						}
        }break;
        case FIRC_MODE_RUN:
        {					
						ramp_calc(&Fric_speed,0.002f,HIGH_SPEED,HIGH_SPEED,0);
						
//						fric[0].spd_ref = -Fric_speed.out;
//						fric[1].spd_ref = Fric_speed.out;	
						fric[0].spd_ref = Fric_speed.out;
				    	fric[1].spd_ref = -Fric_speed.out;
						for(a=0;a<2;a++)
						{	
						    fric[a].spd_fdb = fric_motor[a].speed_rpm;
                            fric[a].current = pid_calc(&fric_spd_pid[a],fric[a].spd_fdb ,fric[a].spd_ref );							               
							fric_motor[a].tx_current = fric[a].current;
						}
        }break;
        default: break;
    }
//    osSignalSet(can_msg_send_task_t, FRIC_MOTOR_MSG_SEND);
}
