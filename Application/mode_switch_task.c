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
#include "prot_sbus.h"
static void unlock_init(void);
static void sw1_mode_handler(void);
static void rc_abnormal_proess(void);
 
/* ϵͳ״̬�� */
ctrl_mode_e ctrl_mode;

/* �Ӿ�״̬�� */
//vision_mode_e vision_mode;

/*ң������һ�ε����ݡ�������һЩ���ؼ��*/
static uint8_t vtm_last_pause=0;
/* ������־ */
uint8_t lock_flag = 0;

/*ң����ʧ����ͼ����־λ*/
uint8_t RC_FAIL_KEYBOARD_FLAG;

/* PIT�������� */
_gimbal_ctrl_ gimbal_ctrl_mode;

/* ϵͳģʽ�л������� */
void mode_switch_task(void const *argu)
{
	ctrl_mode   = PROTECT_MODE;
  lock_flag = 0;
	RC_FAIL_KEYBOARD_FLAG=0;
	
    for(;;)
    {
//			if( Game_State.game_progress != 0)
//				lock_flag = 1;

		if (wbus_rx_cnt > 0)  /* SBUS有数据，直接拨杆决定模式，不走remote_fusion（临时方案，仅供底盘联调用） */
		{
			switch (wbus_rc.data.sw1)
			{
				case 2: ctrl_mode = REMOTER_MODE; break;   /* 拨杆上 */
				case 1: ctrl_mode = PROTECT_MODE; break;   /* 拨杆中 */
				case 0: ctrl_mode = KEYBOARD_MODE; break;  /* 拨杆下 */
				default: break;
			}
		}
////			#if NEW_RC
		else if(remote_fusion.current_source == REMOTE_SOURCE_VTM)
		{				
			vtm_safe_lock();
			
			if( !lock_flag  )
			{
				vtm_unlock_init();  //��������
			}
	
			else if( status.remote)//����Ⲧ����ɾ��imu���
			{
				vtm_mode_handler();  //�����󲦸��л�ϵͳģʽ
			}
			
		}
			
		else if(remote_fusion.current_source == REMOTE_SOURCE_DR16)
		{
			if( !lock_flag  )
			{
				unlock_init();  //��������
			}
				
			if( status.remote)//����Ⲧ����ɾ��imu���
			{
				sw1_mode_handler();  //�����󲦸��л�ϵͳģʽ
			}
			else
			{
				rc_abnormal_proess();  //ң����ʧ������
			}
		}
		/*���ֵƱ仯*/
			fly_led_status();
		if(key_scan_clear(KB_V)) {
				dm_set_cmd(&dm3507_motor,DM_ENABLE_CMD);
		}
				wdg_user_set_bit(WDG_BIT_TASK_MODES);
		
		
        osDelay(5);				
    }
}



/* ң����ʧ���������� */
static void rc_abnormal_proess(void)
{

	  if(status.remote)
		{
			return;//ң�������ȼ���ߣ������ң���������޼��̽���
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
//	shoot.firc_mode = FIRC_MODE_RUN;  //���ֿ���Ħ����
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

/* ϵͳģʽ�л����� */
uint8_t vision_flag;
ctrl_mode_e last_vision_flag ;

/*===============================================��ң�أ�DT7�����뿪ʼ===========================*/
static void sw1_mode_handler(void)  //�ɲ���1����ϵͳģʽ�л�����Ҫ����̨�����̺ͷ�����
{
	last_vision_flag = ctrl_mode;
    switch( remote_fusion.sw.sw1 )
    {
			
			
        case RC_UP:
        {
						if( remote_fusion.channel.ch4 <= -600 )//�󲦸��²�//��ͨ����
						{
							ctrl_mode = VISION_MODE;
											vision_mode = vMODE_AUTO;
						}

						else
						{
										ctrl_mode = REMOTER_MODE;  //����//ң��ģʽ
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

						if( remote_fusion.mouse.right == 1 )  //����Ҽ���ס���Ӿ�
						{
							ctrl_mode = VISION_MODE;
						}
						else
						{
								ctrl_mode = KEYBOARD_MODE;  //����ģʽ
								vision_mode = vMODE_AUTO;
						}			
        }
        break;
        default: break;
	}
		
	
	
}


	
	


//2025��5��16������ ���ڽ�ͼ����·�Ľ�����ز������������������Ŀǰ���뵽�Ŀ�����������ͼ����·����÷���
void Rc_Fail_mode_handler(void)  //�ɲ���1����ϵͳģʽ�л�����Ҫ����̨�����̺ͷ�����
{
if(remote_fusion.keyboard.bit.CTRL)	//��ֹ�󴥣�Ҫ��סCTRL�Ž�������л�ctrl mode����
{
	     if(remote_fusion.keyboard.bit.R)// r for remote ����ʵ�����л�ģʽ��ң����ʧ����ʱ����õģ�����������ʵû���壬������
        {
						if( remote_fusion.channel.ch4 <= -600 )//�󲦸��²�//��ͨ����
						{
							ctrl_mode = VISION_MODE;
											vision_mode = vMODE_AUTO;
						}

						else
						{
										ctrl_mode = REMOTER_MODE;  //����//ң��ģʽ
						}
 
        }
				
				
	     if(remote_fusion.keyboard.bit.S)//s for safe=protect
        {
            ctrl_mode = PROTECT_MODE;          		   
        }
				
				
	     if(remote_fusion.keyboard.bit.B)//b for keyboard
        {

						if( remote_fusion.mouse.right == 1 )  //����Ҽ���ס���Ӿ�
						{
							ctrl_mode = VISION_MODE;
						}
						else
						{
								ctrl_mode = KEYBOARD_MODE;  //����ģʽ
								vision_mode = vMODE_AUTO;
						}			
        }
}
}



/* �������� */
static void unlock_init(void)
{
    if( remote_fusion.sw.sw1 == RC_MI && remote_fusion.sw.sw2 == RC_UP )//�󲦸˾��У��Ҳ�������
    {
        if( remote_fusion.channel.ch4 <= -600 && remote_fusion.channel.ch3 >= 600 && remote_fusion.channel.ch1 <= -600 && remote_fusion.channel.ch2  <= -600 )
        {
            lock_flag = 1;  //�ڰ�
        }
		else  if( remote_fusion.channel.ch4 >= 600 && remote_fusion.channel.ch3 <= -600 && remote_fusion.channel.ch1 >= 600 && remote_fusion.channel.ch2 >= 600 )
        {
            lock_flag = 2;
			
        }
    }
		
		/*2025��5��16������  ң��������ʱ����û����ң�صĿ����߼�����û�в���ϵͳ�������ݵĻ���û�취����lock_flag�� ���ԣ�����ϵͳ�������ݣ����룬ͬ����ʵ��ң�������߼�*/
		else if( remote_fusion.keyboard.bit.Q  )//Q �� Eͬʱ�� �൱��ң�����ڰ�	  &&rc.kb.bit.E		
		{
            lock_flag = 1;  //�ڰ�
		}
		/**/
	
}
/*===============================================��ң�أ�DT7���������===========================*/



///*===============================================��ң�أ�N3�����뿪ʼ===========================*/
void vtm_mode_handler(void)  //�ɲ���1����ϵͳģʽ�л�����Ҫ����̨�����̺ͷ�����
{
	  last_vision_flag = ctrl_mode;
	
    switch(remote_fusion.sw.sw1)
    {
			
		  case RC_UP://����� ң��ģʽ
			{
					if( remote_fusion.channel.ch4 <= -660 )//�󲦸��²�//����
					{
						ctrl_mode = VISION_MODE;
						vision_mode = vMODE_AUTO;
					}				
					else
					{
						ctrl_mode = REMOTER_MODE;  //����//ң��ģʽ
					}
			}
			break;
				
			case RC_MI://�м� ����ģʽ
			{
					ctrl_mode = PROTECT_MODE;		 
			}break;
       
			case RC_DN://���ұ� ����ģʽ
			{
				if( remote_fusion.mouse.right == 1 )  //����Ҽ���ס���Ӿ�//Ĭ��������Ԥ��
				{
					ctrl_mode = VISION_MODE;
				}
				else
				{
					ctrl_mode = KEYBOARD_MODE;  //����ģʽ
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

/* �������� */
void vtm_unlock_init(void)
{
	ctrl_mode = PROTECT_MODE;	//�������ȥ�����������һֱPROTECT_MODE;	
	
    if(remote_fusion.sw.sw1 == RC_MI)//ģʽ�л����ش������ ����ģʽ
    {
        if( remote_fusion.channel.ch1 == -660 && remote_fusion.channel.ch3 == 660 && remote_fusion.channel.ch2 == -660 && remote_fusion.channel.ch4 == -660 )
        {
            lock_flag = 1;  //�ڰ�
        }
        else if( remote_fusion.channel.ch1 == 660 && remote_fusion.channel.ch3 == -660 && remote_fusion.channel.ch2 == -660 && remote_fusion.channel.ch4== -660 )
        {
            lock_flag = 2;			
        }
    }

		/*2025��5��16������  ң��������ʱ����û����ң�صĿ����߼�����û�в���ϵͳ�������ݵĻ���û�취����lock_flag�� ���ԣ�����ϵͳ�������ݣ����룬ͬ����ʵ��ң�������߼�*/
		else if( remote_fusion.keyboard.bit.Q  )//Q �� Eͬʱ�� �൱��ң�����ڰ�	  &&rc.kb.bit.E		
		{
            lock_flag = 1;  //�ڰ�
		}
		/**/
}


/* ң�������������� */
void vtm_safe_lock(void)
{			
	if( vtm_last_pause==0 && vtm_rc.pause==1)//��ͣ����������
	{
		ctrl_mode=PROTECT_MODE;
		lock_flag=0;//��Ҫ���½���		 
	}
	
	vtm_last_pause=vtm_rc.pause;
}

/*===============================================��ң�أ�N3���������===========================*/

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
