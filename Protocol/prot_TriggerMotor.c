/**
  * @file bsp_TriggerMotor.c
  * @version 1.2
  * @date  2023..14
	*
  * @brief 1.0：平平无奇的拨盘代码
	*        1.1：加了退弹和一键reload
	*				 1.2：改了单发的逻辑，现在是低频单发
	*
  *	@author  空中组硬件写的
  *
  */
#include "prot_TriggerMotor.h"
#include "shoot_task.h"
#include "can_comm.h"
#include "pid.h"
#include "gimbal_task.h"
#include "status_task.h"
#include "remote_msg.h"
#include "mode_switch_task.h"
#include "control_def.h"
#include "string.h"
#include "remote_msg.h"
#include "prot_vision.h"
#include "prot_T_imu.h"
#include "prot_rm_motor.h"
#include "prot_dr16.h"
extern shoot_t shoot;

static void TriggerMotor_pidcal(void);

extern TaskHandle_t can_msg_send_task_t;


/*********************************************************************/
//PID结构体
pid_t pid_trigger_ecd;
pid_t pid_trigger_spd;

//以下是卡弹退弹相关
uint8_t back_flag_tri = 0;
uint16_t back_flag;
uint16_t back_flag_cnt;
uint16_t last_back_flag;
uint16_t err_flag;
uint16_t err_time;
uint32_t last_tri_ecd;

//2025年5月15日 弹频变换相关变量
uint8_t SHOOT_PERIOD_CHANGE_CNT;
uint8_t SHOOT_PERIOD_CHANGE_FLAG;
/*********************************************************************/

trigger_t trigger;


uint8_t AMMO_STUCK_FLAG= 0;                 //卡弹标志位 1为卡弹 0为不卡
uint8_t SINGLE_AMMO_STUCK_BACK_FLAG= 0;          //用于单发卡弹退弹只退一次的逻辑判断
uint8_t STIR_SINGLE_SHOT_FLAG;
uint8_t STIR_SERIES_SHOT_FLAG;
uint16_t reload_ammo_cnt=0;	//装子弹数量
//static uint8_t  shoot_enable = 1;    //单发使能标志
uint16_t test_Trigger=6800;//外八解锁的时候改速度
uint32_t SINGLE_STUCK_CNT		= 0;                   //单发卡弹计数
uint32_t SERIES_STUCK_CNT		= 0;                   //连发卡弹计数
 
 
/**
  * @name     TriggerMotor_init
  * @brief    2006初始化
  * @param    None
  * @retval   None
  * @note     速度环 2  0.003  20 单发很好
						
  */

void TriggerMotor_init(void)
{
    PID_struct_init(&pid_trigger_ecd, 18000, 8000,
                    0.1, 0, 0,0,0,0,Integral_Limit);
    PID_struct_init(&pid_trigger_spd, 10000, 8000,
                    5.0, 0.005, 0,0,0,0,Integral_Limit);
	  
	  SHOOT_PERIOD_CHANGE_CNT=0;
}

static void TriggerMotor_pidcal(void)
{
    shoot.pid.trigger_ecd_fdb = trigger_motor.total_ecd;
    pid_calc(&pid_trigger_ecd, shoot.pid.trigger_ecd_fdb, shoot.pid.trigger_ecd_ref);
    shoot.pid.trigger_spd_ref = pid_trigger_ecd.pos_out;  //位置环

    shoot.pid.trigger_spd_fdb = trigger_motor.speed_rpm;
    pid_calc(&pid_trigger_spd, shoot.pid.trigger_spd_fdb, shoot.pid.trigger_spd_ref);
    shoot.current = pid_trigger_spd.pos_out;  //速度环

    trigger_motor.tx_current = shoot.current;
}


///*单发卡弹判断函数：single_shoot_enable通过则进入判断。单发模式下，若ecd_err大于4发的位置，则视为卡弹。 即4次单发请求通过，拨弹盘都没转超过0.5发.注意，该函数在单发ecd+=后被执行，所以该函数内的ecd_err必须至少大于1发的位置，否则必定误判*/
//static void single_ammo_stuck_judge(void) 
//{
////					if(ABS(shoot.pid.trigger_ecd_error) > 0.5f * TRIGGER_MOTOR_ECD)
////					{
////							SINGLE_STUCK_CNT ++ ;
////							if(SINGLE_STUCK_CNT >=4)//3次拨弹盘不到位，则视为卡弹
////							{
////								AMMO_STUCK_FLAG = 1;//置位单发卡弹退弹标志位，用于后面退一发
////								SINGLE_AMMO_STUCK_BACK_FLAG=1;
////							}
////					}
//	          if(ABS(shoot.pid.trigger_ecd_error) > 4.0f * TRIGGER_MOTOR_ECD)
//					{
////							SINGLE_STUCK_CNT ++ ;
////							if(SINGLE_STUCK_CNT >=4)//3次拨弹盘不到位，则视为卡弹
////							{
//								AMMO_STUCK_FLAG = 1;//置位单发卡弹退弹标志位，用于后面退一发
//								SINGLE_AMMO_STUCK_BACK_FLAG=1;
////							}
//					}
//					else
//					{
//						AMMO_STUCK_FLAG = 0;
//					}	
//}

///*单发复位函数 2025年4月21日记得今天加图传链路后完善此代码*/
//static uint8_t single_shoot_reset(void) 
//{
////    return (
////							(   (rc.mouse.l)==0  &&  ctrl_mode==VISION_MODE  &&  vsn_drive.shoot_enable )    //视觉模式下：松开鼠标左键&视觉允许发射，才复位
////							|| (ABS(rc.ch5) < 10 && ctrl_mode == REMOTER_MODE)                             //遥控模式下：拨轮松开，回到（接近）原位，才复位
////							|| ((rc.mouse.l) == 0 && ctrl_mode == KEYBOARD_MODE)                       //键盘模式下：松开鼠标左键，才复位
////           );
//}

//单发复位enable，且遥控发送单发请求
uint8_t single_shoot_enable(void) {
	 #if NEW_RC//2025年7月3日 新遥控由于没有第二个拨杆，所以不再写遥控触发的单发了。后续如果视觉要打符，再想怎么写
//		return(
//        trigger.trigger_enable
//        && (new_rc.mouse_left || new_rc.trigger >= 1)
//        && ABS(shoot.pid.trigger_ecd_error) < TRIGGER_MOTOR_ECD
//    );
	#else
				return(
        trigger.trigger_enable
        && (rc.mouse.l || rc.ch5 > 500)
        && ABS(shoot.pid.trigger_ecd_error) < TRIGGER_MOTOR_ECD
    );
	#endif
}

uint8_t series_shoot_enable(void) {
	
//	  #if NEW_RC 
	  
//	if(remote_online.NEW_OLD_RC_FLAG == 1)
//	{
//		return(
//        ((ctrl_mode == REMOTER_MODE)// && vision.shoot_enable
//            || (ctrl_mode == KEYBOARD_MODE && new_rc.mouse_left) 
//            || (ctrl_mode == VISION_MODE && vsn_drive.shoot_enable)
//        )
//        && (shoot.frequency_cnt  >= TRIGGER_PERIOD)  /* 射频控制 */
//		&& ABS(shoot.pid.trigger_ecd_error) < 0.9f * TRIGGER_MOTOR_ECD
//    );
//	}
//		#else
		
//		else if(remote_online.NEW_OLD_RC_FLAG == 0)
//		{
			return(
        ((ctrl_mode == REMOTER_MODE)// && vision.shoot_enable
            || (ctrl_mode == KEYBOARD_MODE && rc.mouse.l) 
            || (ctrl_mode == VISION_MODE && vision.shoot_enable)
        )
        && (shoot.frequency_cnt  >= TRIGGER_PERIOD)  /* 射频控制 */
    );
//	}
//    #endif
}



/**
  * @name     TriggerMotor_control
  * @brief    射频控制
  * @param    None
  * @retval   None
  * @note     2025年3月31日抄步兵
						
  */
void TriggerMotor_control(void)
{
	if(RC_FAIL_KEYBOARD_FLAG==1)
	{
//		shoot.stir_mode=STIR_MODE_SERIES;
	}
	
      switch (shoot.stir_mode) {
        case STIR_MODE_PROTECT: 
				{ //拨盘保护模式，保持惯性，无力
            shoot.frequency_cnt = 0; //计时变量置0，打出当前一发，禁止
            shoot.trigger_period = 0;
						shoot.pid.trigger_ecd_ref = trigger_motor.total_ecd;//让其原地停下，防干碎拨盘
						shoot.pid.trigger_ecd_fdb = trigger_motor.total_ecd;            
					  pid_trigger_spd.iout  = 0;
				trigger_motor.tx_current = 0;
            break;
        }

        case STIR_MODE_SINGLE: 
					{ 
					 shoot.pid.trigger_ecd_error = shoot.pid.trigger_ecd_ref - shoot.pid.trigger_ecd_fdb;
//           if(!AMMO_STUCK_FLAG)//非卡弹标志位 即不卡
//				   { 					
//							/*执行单发逻辑*/
//							if (single_shoot_reset())
//							{//单次发射后等待复位
//									trigger.trigger_enable = T_ENABLE;
//							}
//							if (single_shoot_enable())
//							{ //已复位并且单发条件判断通过，则ecd+=，同时判断是否卡弹
//									trigger.trigger_enable = T_DISABLE;//已经执行ecd+=，需要再次复位才允许再次ecd+=
//								  shoot.pid.trigger_ecd_ref += TRIGGER_MOTOR_ECD;
//								  single_ammo_stuck_judge();//执行卡弹判断
//							}
//				   }
//					else//若AMMO_STUCK_FLAG为1，视为卡弹，执行退弹操作
//					{
//						if(SINGLE_AMMO_STUCK_BACK_FLAG==1)//因为这个else里面要等待退一发后电机到位，所以这个ecd-=要单独出来保证只执行一次
//						{
//							shoot.pid.trigger_ecd_ref -= TRIGGER_MOTOR_ECD; //退一发
//							SINGLE_AMMO_STUCK_BACK_FLAG=0;//ecd-=一发后，置零单发卡弹退弹标志位，防止多次ecd-=	
//						}
//						
//						
//						if(rc.mouse.l || rc.ch5 > 500)//ecd-=后，AMMO_STUCK_FLAG依然为1，依然进这个else，此时如果继续请求单发，则等待PID控制器完成ecd-=，用CNT来模拟计时器
//						{
//							SINGLE_STUCK_CNT++;
//							if(SINGLE_STUCK_CNT>=1000)//ecd_ref退一发之后，本来是还需要判断能不能退到位的，但没测过，不知道退1发？还是退0.5发？还是退0.1发的位置就够了？ 所以一发退不到位也是正常的，故不判断，只搞一个CNT，用来计时，计几次够呢？ 不知道，随便填个1000，即大概1秒
//							{
//								SINGLE_STUCK_CNT = 0;//清零，下一次卡弹再用
//								AMMO_STUCK_FLAG=0;
//							}
//						}
//					}
					TriggerMotor_pidcal();
            break;
        }
				
        case STIR_MODE_SERIES: 
					{ //拨盘连发模式，连续开枪请求，连续响应
            shoot.pid.trigger_ecd_error = shoot.pid.trigger_ecd_ref - shoot.pid.trigger_ecd_fdb;
						
					if(!back_flag_tri)//如果卡弹了，就不再接收发射指令，专心完成退弹操作先
				  {
//						if(remote_online.NEW_OLD_RC_FLAG == 1)
//						{
////						#if NEW_RC
//						if( (new_rc.mouse_left || new_rc.trigger == 1) )
//						{					
//							  /*2025年5月15日 加个射频变化控制 云台手可以通过摁住shift来提高射频*/
//							  if(new_rc.key_SHIFT==1 || rc.ch3 <=364)
//								{
//									SHOOT_PERIOD_CHANGE_CNT+=2;
//									SHOOT_PERIOD_CHANGE_FLAG=1;

//								}
//								else
//								{
//						   		SHOOT_PERIOD_CHANGE_FLAG=0;

//								}
//								if(SHOOT_PERIOD_CHANGE_CNT>=4)
//								{
//									SHOOT_PERIOD_CHANGE_CNT=0;
//									shoot.frequency_cnt+=1;
//									SHOOT_PERIOD_CHANGE_FLAG=1;
//								}
//						
//						/*以上代码，不用时，直接注释*/			
//									shoot.frequency_cnt++;
//							  /*射频变换代码结束，相关代码还有：最上面CNT定义，init函数CNT清零*/
//									if (series_shoot_enable() && back_flag_tri ==0) 
//									{ 
//										STIR_SERIES_SHOT_FLAG++;
//										shoot.frequency_cnt = 0;
//										shoot.pid.trigger_ecd_ref += TRIGGER_MOTOR_ECD;/* 拨一颗子弹 */							
//									}
//						}
//						else
//						{
//							shoot.pid.trigger_ecd_ref = trigger_motor.total_ecd;//让其原地停下，防干碎拨盘
//							shoot.pid.trigger_ecd_fdb = trigger_motor.total_ecd;   
//						}
////						
////					#else
//					}
//						else if(remote_online.NEW_OLD_RC_FLAG == 0)
//						{
							if( (rc.mouse.l || rc.ch5 >= 500) )
						{					
							  /*2025年5月15日 加个射频变化控制 云台手可以通过摁住shift来提高射频*/
							  if(rc.kb.bit.SHIFT==1 || rc.ch3 <=-600)
								{
									SHOOT_PERIOD_CHANGE_CNT+=2;
									SHOOT_PERIOD_CHANGE_FLAG=1;

								}
								else
								{
						   		SHOOT_PERIOD_CHANGE_FLAG=0;

								}
								if(SHOOT_PERIOD_CHANGE_CNT>=4)
								{
									SHOOT_PERIOD_CHANGE_CNT=0;
									shoot.frequency_cnt+=1;
									SHOOT_PERIOD_CHANGE_FLAG=1;
								}
						
						/*以上代码，不用时，直接注释*/			
									shoot.frequency_cnt++;
							  /*射频变换代码结束，相关代码还有：最上面CNT定义，init函数CNT清零*/
									if (series_shoot_enable() && back_flag_tri ==0) 
									{ 
										STIR_SERIES_SHOT_FLAG++;
										shoot.frequency_cnt = 0;
										shoot.pid.trigger_ecd_ref += TRIGGER_MOTOR_ECD;/* 拨一颗子弹 */							
									}
						}
						else
						{
							shoot.pid.trigger_ecd_ref = trigger_motor.total_ecd;//让其原地停下，防干碎拨盘
							shoot.pid.trigger_ecd_fdb = trigger_motor.total_ecd;   
						}
						
//						#endif
//					}//else if 的括号
				 }

            /* 先卡弹判断：若电机堵转，电流会增大 + 转速会降低*/
						if(ABS((int)trigger_motor.rx_current)>10000 && ABS(trigger_motor.speed_rpm)<500)//
							{
								back_flag_cnt++;
								trigger_motor.tx_current = 0;
							}
							
						/*若持续卡弹，则进入退弹*/
             if(back_flag_cnt > 200)
            {          
                shoot.pid.trigger_ecd_ref = trigger_motor.total_ecd;//对齐当前拨盘位置（拨盘的位置环一般都跟不上，会拉很远，所以要重新对齐）
                last_tri_ecd = trigger_motor.total_ecd;//记录当前拨盘位置，后续用于退出退弹的判断
                back_flag_tri = 1;//置位退弹标志位 表面拨盘退弹中
							  back_flag = 1;//置位一次back_flag，后续只-=一次
								back_flag_cnt = 0;//清除卡弹计数，防止重复进入该if
            }   
						
						//每次卡弹，只-=一次ref
						if(back_flag==1)
						{
							  shoot.pid.trigger_ecd_ref -= 2.0f * TRIGGER_MOTOR_ECD;//退两发的位置
							  back_flag = 0;//清除，防止重复-=ref
						}
						
						/*等待退弹执行结束*/
						if( back_flag_tri == 1 )//退弹标志位 表明正在执行退弹
						{
							 err_time += SHOOT_PERIOD;//退弹执行计时 或超时，则直接退出退弹而不再等拨盘到位（退弹时阻力依然很大，拨盘不到位也正常）
							
							//退弹结束的依据：反转到位（2.0发）或超时（执行200次发射任务）
							if((err_time >= 200) || (trigger_motor.total_ecd == (last_tri_ecd - 2.0*TRIGGER_MOTOR_ECD)) )
							{
								  err_time = 0;//清除退弹执行时间 防止影响下一次退弹
								  back_flag_tri=0;//清除退弹标志位 
							}
					  }
						
						/*卡弹退弹代码结束*/
						
						TriggerMotor_pidcal();						
            break;
        }
				
        default: {break;}
    }
//    osSignalSet(can_msg_send_task_t, TRIGGER_MOTOR_MSG_SEND);
}

