  #define __SHOOT_TASK_GLOBALS

#include "shoot_task.h"
#include "control_def.h"
#include "cmsis_os.h"
#include "usart.h"
#include "prot_FricMotor.h"
#include "prot_TriggerMotor.h"
#include "prot_rm_judge.h"
#include "prot_vision.h"
#include "remote_msg.h"
#include "comm_task.h"
//#include "status_task.h"
#include "mode_switch_task.h"
#include "pid.h"
#include "prot_dr16.h"

extern robot_status_t	robot_status;
extern game_status_t game_status;
//extern aerial_robot_energy_t					Aerial_Robot_Energy;
shoot_t shoot;

static void shoot_mode_sw(void);

void shoot_task(void const *argu)
{
    uint32_t mode_wake_time = osKernelSysTick();
    for(;;)
    {
        shoot_mode_sw();  //射击模式切换
        FricMotor_Control();	//摩擦轮电机控制
        TriggerMotor_control();	//拨弹电机控制
//	    wdg_user_set_bit(WDG_BIT_TASK_SHOOT);
        osDelayUntil(&mode_wake_time, SHOOT_PERIOD);
    }
}

void shoot_init(void)
{
    FricMotor_init();   //摩擦轮初始化
    TriggerMotor_init();//拨盘初始化
    /* 发射器模式初始化 */
    shoot.firc_mode     = FIRC_MODE_STOP;
    shoot.stir_mode     = STIR_MODE_PROTECT;
    /* 枪管参数初始化 */
    shoot.trigger_period = TRIGGER_PERIOD;
	/*新遥控相关开始*/
	shoot.shoot_mode_CNT=0;
	/*新遥控相关结束*/
}


static void shoot_mode_sw(void)
{ 
	
////		#if NEW_RC
//		if(remote_online.NEW_OLD_RC_FLAG == 1)
//			{
//    /* 模式切换 */

//		 switch( new_rc.mode_sw )
//    {
//        case RC_MODE_C://保护
//        {
//            shoot.firc_mode = FIRC_MODE_STOP;
//            shoot.stir_mode = STIR_MODE_PROTECT;
//			shoot.shoot_mode_CNT=0;
//        }
//        break;
//				
//        case RC_MODE_N://遥控
//        {
//					shoot.FN2=new_rc.fn_2;
//						if(shoot.FN2==1 && shoot.last_FN2==0)//上升沿
//						{
//								 shoot.shoot_mode_CNT++;
//						}
//						
//						if(shoot.firc_mode == FIRC_MODE_RUN && shoot.FN2==1 && shoot.last_FN2==0)
//						{
//							shoot.firc_mode = FIRC_MODE_STOP;
//							shoot.stir_mode = STIR_MODE_PROTECT;
//							shoot.shoot_mode_CNT = 0;
//						}
//						
//						if(shoot.shoot_mode_CNT>=5)
//						{
//							shoot.firc_mode = FIRC_MODE_RUN;
//							shoot.stir_mode = STIR_MODE_SERIES;
//						}
//					shoot.last_FN2=shoot.FN2;

//        }
//        break;
//       
//        case RC_MODE_S://键鼠
//        {
//					 shoot.firc_mode = FIRC_MODE_RUN;
//					 shoot.stir_mode = STIR_MODE_SERIES;
//        }
//        break;
//        default: break;
//    }
//			

//			
////		#else
//	  }
//			
//		else if(remote_online.NEW_OLD_RC_FLAG == 0)
//		{
		
		    /* 模式切换 */
    switch( rc.sw1 )
    {
        case RC_MI:
        {
            shoot.firc_mode = FIRC_MODE_STOP;
            shoot.stir_mode = STIR_MODE_PROTECT;
        }
        break;
				
        case RC_UP:
        {
            /* 摩擦轮和拨盘模式切换 */
            switch( rc.sw2 )
            {
                case RC_UP:
                {
                    shoot.firc_mode = FIRC_MODE_STOP;
                    shoot.stir_mode = STIR_MODE_PROTECT;
                }
                break;
                case RC_MI:
                {
                    shoot.firc_mode = FIRC_MODE_RUN;  //开启摩擦轮
                    shoot.stir_mode = STIR_MODE_SINGLE;
                }
                break;
                case RC_DN:
                {
                    shoot.firc_mode = FIRC_MODE_RUN;  //保持开启摩擦轮
					          shoot.stir_mode = STIR_MODE_SERIES;
                }
                break;
                default: break;
            }
        }
        break;
//        
        case RC_DN:
        {
					 shoot.firc_mode = FIRC_MODE_RUN;
					 shoot.stir_mode = STIR_MODE_SERIES;
        }
        break;
        default: break;
    }
		
//	}


}

