#include "mode_switch_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "can_comm.h"
#include "control_def.h"
#include "prot_TriggerMotor.h"
#include "remote_msg.h"
#include "cmsis_os.h"
#include "prot_vision.h"
#include "status_task.h"
#include "prot_rm_judge.h"
#include "prot_dr16.h"
#include "prot_new_rc.h"
#include "prot_remote_fusion.h"
#include "prot_ws2812.h"
#include "prot_dm_motor.h"
static void unlock_init(void);
static void sw1_mode_handler(void);
static void rc_abnormal_proess(void);
 
/* 系统状态机 */
ctrl_mode_e ctrl_mode;

/* 视觉状态机 */
//vision_mode_e vision_mode;

/*遥控器上一次的数据——用于一些边沿检测*/
static uint8_t vtm_last_pause=0;
/* 解锁标志 */
uint8_t lock_flag = 0;

/*遥控器失联切图传标志位*/
uint8_t RC_FAIL_KEYBOARD_FLAG;

/* PIT锁死开关 */
_gimbal_ctrl_ gimbal_ctrl_mode;

/* 系统模式切换任务函数 */
void mode_switch_task(void const *argu)
{
	ctrl_mode   = PROTECT_MODE;
  lock_flag = 0;
	RC_FAIL_KEYBOARD_FLAG=0;
	
    for(;;)
    {
//			if( Game_State.game_progress != 0)
//				lock_flag = 1;
			
////			#if NEW_RC
		if(remote_fusion.current_source == REMOTE_SOURCE_VTM)
		{				
			vtm_safe_lock();
			
			if( !lock_flag  )
			{
				vtm_unlock_init();  //解锁操作
			}
	
			else if( status.remote)//如果测拨盘则删掉imu检测
			{
				vtm_mode_handler();  //根据左拨杆切换系统模式
			}
			
		}
			
		else if(remote_fusion.current_source == REMOTE_SOURCE_DR16)
		{
			if( !lock_flag  )
			{
				unlock_init();  //解锁操作
			}
				
			if( status.remote)//如果测拨盘则删掉imu检测
			{
				sw1_mode_handler();  //根据左拨杆切换系统模式
			}
			else
			{
				rc_abnormal_proess();  //遥控器失联处理
			}
		}
		/*飞手灯变化*/
			fly_led_status();
		if(key_scan_clear(KB_V)) {
				dm_set_cmd(&dm3507_motor,DM_ENABLE_CMD);
		}
				wdg_user_set_bit(WDG_BIT_TASK_MODES);
		
		
        osDelay(5);				
    }
}



/* 遥控器失联处理函数 */
static void rc_abnormal_proess(void)
{

	  if(status.remote)
		{
			return;//遥控器优先级最高，如果有遥控器，就无键盘解锁
		}
    RC_FAIL_KEYBOARD_FLAG=1;

//		rc.mouse.x=Remote_Control.mouse_x;
//		rc.mouse.y=Remote_Control.mouse_y;
//		rc.mouse.z=Remote_Control.mouse_z;
//		rc.mouse.l=Remote_Control.left_button_down;
//		rc.mouse.r=Remote_Control.right_button_down;
//		 
//		rc.kb.key_code=Remote_Control.keyboard_value;
//		
//		
//	shoot.firc_mode = FIRC_MODE_RUN;  //保持开启摩擦轮
//	shoot.stir_mode = STIR_MODE_SERIES;
//										
//										
//		if(rc.mouse.r)
//		{
//	  ctrl_mode=VISION_MODE;
//		}
//		else 
//		{
//	  ctrl_mode=KEYBOARD_MODE;
//		}
		
	 Rc_Fail_mode_handler();


}

/* 系统模式切换函数 */
uint8_t vision_flag;
ctrl_mode_e last_vision_flag ;

/*===============================================旧遥控（DT7）代码开始===========================*/
static void sw1_mode_handler(void)  //由拨杆1决定系统模式切换，主要是云台、底盘和发射器
{
	last_vision_flag = ctrl_mode;
    switch( remote_fusion.sw.sw1 )
    {
			
			
        case RC_UP:
        {
						if( remote_fusion.channel.ch4 <= -600 )//左拨杆下拨//普通自瞄
						{
							ctrl_mode = VISION_MODE;
											vision_mode = vMODE_AUTO;
						}

						else
						{
										ctrl_mode = REMOTER_MODE;  //回中//遥控模式
						}
 
        }
        break;
				
				
        case RC_MI:
        {
            ctrl_mode = PROTECT_MODE;
            		   
        }
        break;
				
				
        case RC_DN:
        {

						if( remote_fusion.mouse.right == 1 )  //鼠标右键按住开视觉
						{
							ctrl_mode = VISION_MODE;
						}
						else
						{
								ctrl_mode = KEYBOARD_MODE;  //键盘模式
								vision_mode = vMODE_AUTO;
						}			
        }
        break;
        default: break;
	}
		
	
	
}


	
	


//2025年5月16日添加 用于将图传链路的解锁相关操作适配进来，这是我目前能想到的快速完美兼容图传链路的最好方法
void Rc_Fail_mode_handler(void)  //由拨杆1决定系统模式切换，主要是云台、底盘和发射器
{
if(remote_fusion.keyboard.bit.CTRL)	//防止误触，要按住CTRL才进入键盘切换ctrl mode功能
{
	     if(remote_fusion.keyboard.bit.R)// r for remote 但其实键盘切换模式是遥控器失联的时候才用的，所以这里其实没意义，仅保留
        {
						if( remote_fusion.channel.ch4 <= -600 )//左拨杆下拨//普通自瞄
						{
							ctrl_mode = VISION_MODE;
											vision_mode = vMODE_AUTO;
						}

						else
						{
										ctrl_mode = REMOTER_MODE;  //回中//遥控模式
						}
 
        }
				
				
	     if(remote_fusion.keyboard.bit.S)//s for safe=protect
        {
            ctrl_mode = PROTECT_MODE;          		   
        }
				
				
	     if(remote_fusion.keyboard.bit.B)//b for keyboard
        {

						if( remote_fusion.mouse.right == 1 )  //鼠标右键按住开视觉
						{
							ctrl_mode = VISION_MODE;
						}
						else
						{
								ctrl_mode = KEYBOARD_MODE;  //键盘模式
								vision_mode = vMODE_AUTO;
						}			
        }
}
}



/* 解锁函数 */
static void unlock_init(void)
{
    if( remote_fusion.sw.sw1 == RC_MI && remote_fusion.sw.sw2 == RC_UP )//左拨杆居中，右拨杆置上
    {
        if( remote_fusion.channel.ch4 <= -600 && remote_fusion.channel.ch3 >= 600 && remote_fusion.channel.ch1 <= -600 && remote_fusion.channel.ch2  <= -600 )
        {
            lock_flag = 1;  //内八
        }
		else  if( remote_fusion.channel.ch4 >= 600 && remote_fusion.channel.ch3 <= -600 && remote_fusion.channel.ch1 >= 600 && remote_fusion.channel.ch2 >= 600 )
        {
            lock_flag = 2;
			
        }
    }
		
		/*2025年5月16日添加  遥控器断联时，若没有新遥控的控制逻辑，又没有裁判系统键鼠数据的话，没办法控制lock_flag。 所以，裁判系统键鼠数据，必须，同样能实现遥控器的逻辑*/
		else if( remote_fusion.keyboard.bit.Q  )//Q 和 E同时按 相当于遥控器内八	  &&rc.kb.bit.E		
		{
            lock_flag = 1;  //内八
		}
		/**/
	
}
/*===============================================旧遥控（DT7）代码结束===========================*/



///*===============================================新遥控（N3）代码开始===========================*/
void vtm_mode_handler(void)  //由拨杆1决定系统模式切换，主要是云台、底盘和发射器
{
	  last_vision_flag = ctrl_mode;
	
    switch(remote_fusion.sw.sw1)
    {
			
		  case RC_UP://最左边 遥控模式
			{
					if( remote_fusion.channel.ch4 <= -660 )//左拨杆下拨//自瞄
					{
						ctrl_mode = VISION_MODE;
						vision_mode = vMODE_AUTO;
					}				
					else
					{
						ctrl_mode = REMOTER_MODE;  //回中//遥控模式
					}
			}
			break;
				
			case RC_MI://中间 保护模式
			{
					ctrl_mode = PROTECT_MODE;		 
			}break;
       
			case RC_DN://最右边 键鼠模式
			{
				if( remote_fusion.mouse.right == 1 )  //鼠标右键按住开视觉//默认自瞄无预测
				{
					ctrl_mode = VISION_MODE;
				}
				else
				{
					ctrl_mode = KEYBOARD_MODE;  //键盘模式
					vision_mode = vMODE_AUTO;
				}			
			}break;
			
      default: break;
	  }
		
		if((last_vision_flag == VISION_MODE)&&(ctrl_mode != VISION_MODE))
		{	
			vision_flag = 1;
		}

}

/* 解锁函数 */
void vtm_unlock_init(void)
{
	ctrl_mode = PROTECT_MODE;	//如果出不去这个函数，就一直PROTECT_MODE;	
	
    if(remote_fusion.sw.sw1 == RC_MI)//模式切换开关打到最左边 保护模式
    {
        if( remote_fusion.channel.ch1 == -660 && remote_fusion.channel.ch3 == 660 && remote_fusion.channel.ch2 == -660 && remote_fusion.channel.ch4 == -660 )
        {
            lock_flag = 1;  //内八
        }
        else if( remote_fusion.channel.ch1 == 660 && remote_fusion.channel.ch3 == -660 && remote_fusion.channel.ch2 == -660 && remote_fusion.channel.ch4== -660 )
        {
            lock_flag = 2;			
        }
    }

		/*2025年5月16日添加  遥控器断联时，若没有新遥控的控制逻辑，又没有裁判系统键鼠数据的话，没办法控制lock_flag。 所以，裁判系统键鼠数据，必须，同样能实现遥控器的逻辑*/
		else if( remote_fusion.keyboard.bit.Q  )//Q 和 E同时按 相当于遥控器内八	  &&rc.kb.bit.E		
		{
            lock_flag = 1;  //内八
		}
		/**/
}


/* 遥控器紧急进保护 */
void vtm_safe_lock(void)
{			
	if( vtm_last_pause==0 && vtm_rc.pause==1)//暂停按键被按下
	{
		ctrl_mode=PROTECT_MODE;
		lock_flag=0;//需要重新解锁		 
	}
	
	vtm_last_pause=vtm_rc.pause;
}

/*===============================================新遥控（N3）代码结束===========================*/

void fly_led_status(void)
{
	
	if(remote_fusion.keyboard.bit.W || remote_fusion.channel.ch4 > 600) // ????????
    {	update_leds(RGB_NUM, OFF);
		led_range_on(4, 12, PURPLE);
	}
	else if(remote_fusion.keyboard.bit.D || remote_fusion.channel.ch3 > 600) 
	{	update_leds(RGB_NUM, OFF);	
		led_range_on(0, 8, PURPLE);
	}
	else if(remote_fusion.keyboard.bit.S || remote_fusion.channel.ch4 < -600) 
	{	update_leds(RGB_NUM, OFF);		
		led_range_on(0, 4, PURPLE);
		led_range_on(11,16, PURPLE);
	}
	else if(remote_fusion.keyboard.bit.A || remote_fusion.channel.ch3 < -600) 
	{	update_leds(RGB_NUM, OFF);	
		led_range_on(8, 16, PURPLE);
	}
	else if(remote_fusion.keyboard.bit.Q) 
	{	update_leds(RGB_NUM, OFF);	
		led_range_on(0, RGB_NUM, RED);
	}
	else if(remote_fusion.keyboard.bit.E) 
	{	update_leds(RGB_NUM, OFF);	
		led_range_on(0, RGB_NUM, YELLOW);
	}
	else led_double_flash(0, 16, PURPLE, 600, 600);  // 0-7???????????????200ms??200ms
	
	rgb_output_data();
}
