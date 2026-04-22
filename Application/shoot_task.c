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
#include "prot_rm_motor.h"
//#include "status_task.h"
#include "mode_switch_task.h"
#include "pid.h"
#include "prot_dr16.h"
#include "prot_remote_fusion.h"
extern robot_status_t	robot_status;
extern game_status_t game_status;
//extern aerial_robot_energy_t					Aerial_Robot_Energy;
shoot_t shoot;

static void shoot_mode_sw(void);
static void shoot_param_update(void);
void shoot_task(void const *argu)
{
    uint32_t mode_wake_time = osKernelSysTick();
    for(;;)
    {
        shoot_mode_sw();  //射击模式切换
        FricMotor_Control();	//摩擦轮电机控制
        TriggerMotor_control();	//拨弹电机控制
		dji_send_current(&fric_motor[0], &fric_motor[1],&trigger_motor, NULL);
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
	shoot_param_update();
		
		    /* 模式切换 */
    switch( remote_fusion.sw.sw1 )
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
            switch( remote_fusion.sw.sw2 )
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
				if( key_scan_clear(KB_B) ){
					 shoot.firc_mode = FIRC_MODE_RUN;
					 shoot.stir_mode = STIR_MODE_SINGLE;
				}
				else{	 
					 shoot.firc_mode = FIRC_MODE_RUN;
					 shoot.stir_mode = STIR_MODE_SERIES;
				}
        }
        break;
        default: break;
    }
		
}

static void shoot_param_update(void)
{
    //更新裁判系统数据
    if (robot_status.shooter_barrel_heat_limit != 0) {
        shoot.barrel.heat_max = robot_status.shooter_barrel_heat_limit;//枪管热量上限
        shoot.barrel.cooling_rate = robot_status.shooter_barrel_cooling_value;//枪管冷却速率   /*裁判系统热量数据*/
		
    }
	/*模仿裁判系统热量数据*/
//	shoot.barrel.heat_max =120;  //满级 200
//		shoot.barrel.cooling_rate =40;   //满级 120
    //更新模拟裁判系统数据
    shoot.barrel.heat -= shoot.barrel.cooling_rate * SHOOT_PERIOD * 0.001f;  //当前枪管(理论)热量
    if (shoot.barrel.heat < 0) shoot.barrel.heat = 0;
    shoot.barrel.heat_remain = shoot.barrel.heat_max - shoot.barrel.heat;  //当前枪管(理论)剩余热量
//    shoot.barrel.heat_remain = 50;
}
