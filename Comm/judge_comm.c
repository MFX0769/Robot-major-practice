/**
  * @file    judge_send_task.c
  * @version 2.0
  * @date    2024.4.26
	*
  * @brief   UI发送定时器与发送任务调度
	* 
	* @note    串口波特率115200bit/s , 为方便计算传输频率，且保证每一条指令传输完整，将最高频率设为100Hz
	*          
	*   
	*         每次回调能传输数据数 = 115200/1000*10 =1152bit/次（仅为理论值，还要考虑回调执行时间等其他因素）
	*           
	*                              图像       数组长度     单个数据bit位      传输指令条数      总传输位数
	*         对应传输ui次数     DRAW_IMAGINE1    30     *       8         *        4               960
    *                           DRAW_IMAGINE2    45     *       8         *        2               720
	*                           DRAW_IMAGINE5    90     *       8         *        1               720
	*                           DRAW_IMAGINE7    120    *       8         *        1               980
	*                           PRINTF_CODE      60     *       8         *        2               960
	*                           JUDGE_DELETE     17     *       8         *        7               952
  *
  *	@author  ZZK & 佚名搬运工
  *
  */
#include "judge_comm.h"
#include "prot_rm_judge.h"
#include "prot_judge_send.h"
#include "usart_comm.h"
#include "cmsis_os.h"
#include "mode_switch_task.h"
#include "string.h"
#include "prot_FricMotor.h"
#include "prot_TriggerMotor.h"
#include "shoot_task.h"
#include "remote_msg.h"
#include "prot_T_imu.h"
#include "prot_vision.h"
extern robot_status_t robot_status;
extern game_robot_HP_t      RobotHP;
extern game_status_t					Game_status;
extern robot_interactive_receivedata_t		Comm_getdata;
extern Taurus_imu_data_t   imu_data;


extern self_inf_t self_inf;
extern uint8_t ui_seq;

/*准星*/
graphic_data_struct_t background[7];
uint8_t separate1[3] 	= "sp1";
uint8_t separate2[3] 	= "sp2";
uint8_t separate3[3] 	= "sp3";
uint8_t separate4[3] 	= "sp4";
uint8_t separate5[3] 	= "sp5";
/*角度轴*/
uint8_t pit[3]="pit";
uint8_t pit_line[3]="pil";

/*各状态标志位*/
graphic_data_struct_t gim_status_init[7];
uint8_t gim_mode1[3]    ="md1";//外框
uint8_t gim_mode2[3]    ="md2";//内框
uint8_t gim_mode3[3]    ="md3";//选择框
uint8_t fric_mode[3]   ="Frc";
uint8_t vis_mode[3] ="Vis";//视觉模式左括号

/*刻度线*/
graphic_data_struct_t scale_line[7];
uint8_t scale1[3]="sc1";
uint8_t scale2[3]="sc2";
uint8_t scale3[3]="sc3";
uint8_t scale4[3]="sc4";
uint8_t scale5[3]="sc5";

/*刻度线*/
graphic_data_struct_t camera_line[7];
uint8_t camera_line1[3]="cl1";
uint8_t camera_line2[3]="cl2";
uint8_t camera_line3[3]="cl3";
uint8_t camera_line4[3]="cl4";

/*刻度文本*/
int_data_struct_t scale_num[5];
uint8_t sc_num1[3]="sd1";
uint8_t sc_num2[3]="sd2";
uint8_t sc_num3[3]="sd3";
uint8_t sc_num4[3]="sd4";
uint8_t sc_num5[3]="sd5";


/*LOCK_FALG*/
int_data_struct_t lock_flag_ui[1];
uint8_t lock_flag_ui1[3]="lfu";


/*动态部分*/
graphic_data_struct_t gimbal_status[7];

uint8_t vis_draw[3]  ="dra";
uint16_t vir_x,vir_y;




void ui_init_data_packed(void);
void ui_init_data_send(void);
void ui_dynamic_data_pack(void);
uint8_t init_flag;
uint32_t ui_CNT=0;
uint8_t init_cnt=0;
uint8_t ui_init_flag=0;
uint8_t ui_fric, ui_stir, ui_gimbal, ui_gimbal_ctrl;
//判断模式是否改变
#define FRIC_MODE_CHANGE      ui_fric != shoot.firc_mode
#define STIR_MODE_CHANGE      ui_stir != shoot.stir_mode
#define GIMBAL_MODE_CHANGE    ui_gimbal != ctrl_mode
#define CTRL_MODE_CHANGE      ui_gimbal_ctrl != gimbal_ctrl_mode

//ui发送任务
void judge_send_task(void const *argu)
{
    uint32_t thread_wake_time = osKernelSysTick();
	//判断自身ID，选择客户端ID及阵营
//		ui_init_data_packed();
//	
//		/*初始化ui*/
//		for(uint8_t i=0;i<10;i++)
//		{
//			ui_init_data_send();		 
//		}
		
  for(;;)
  {				
//		ui_init_data_packed();
//		/*手动刷新初始化ui*/
////    if(remote_online.NEW_OLD_RC_FLAG	==	1)
////		{
////		if(new_rc.key_R == 1)
////		{
////			ui_init_flag++;
////			for(uint8_t i=0;i<10;i++)
////			{
////			ui_init_data_send();
////				
////				 
////			}
////		}
////	}
////		else if(remote_online.NEW_OLD_RC_FLAG	==	0)
//		if(rc.kb.bit.R == 1)
//		{
//			ui_init_flag++;
//			for(uint8_t i=0;i<10;i++)
//			{
////		ui_init_data_send();
//				
//				 
//			}
//		}
//		if(ui_CNT % 2 ==1)
//		{
////			ui_dynamic_data_pack();
////			ui_sendgraph(DRAW_IMAGINE7,(uint8_t *) gimbal_status, CLIENT_DRAW_7_GRAPHS_CMD_ID);
//		}
//		ui_CNT++;
//			
//		osDelay(30); 
		osDelayUntil(&thread_wake_time, 30);
  }
}

//ui初始化数据打包
void ui_init_data_packed(void)
{
	judge_get_basedata();
	//准星
//	ui_data_packed_line(&background[0],separate1,1,1,8,2,960,440,960,640);//竖线
//	ui_data_packed_line(&background[1],separate2,1,1,8,2,950,520,970,520);//横线
//	ui_data_packed_line(&background[2],separate3,1,1,8,2,950,540,970,540);//横线
//	ui_data_packed_line(&background[3],separate4,1,1,8,2,950,560,970,560);//横线
//	ui_data_packed_line(&background[4],separate5,1,1,8,2,950,500,970,500);//横线
	//云台角度
	ui_data_packed_line(&background[5],pit_line,1,1,5,8,1800,560,1800,800);//角度轴  (40+80)*2
	ui_data_packed_line(&background[6],pit,1,0,6,4,1790,720,1810,720);//实时角度线
	//云台状态机   960,540
	ui_data_packed_rectangle(&gim_status_init[0],gim_mode1,1,1,8,6,840,220,1080,232);//外框
	ui_data_packed_rectangle(&gim_status_init[1],gim_mode2,1,1,8,5,920,220,1000,232);//内框
	ui_data_packed_line(&gim_status_init[2],gim_mode3,1,0,1,8,840,227,1080,227);//滑块
	//发射机构状态  
	ui_data_packed_arc(&gim_status_init[3],fric_mode,1,0,8,2,220,140,960,540,395,395);//Fric
	ui_data_packed_line(&gim_status_init[4],vis_mode,1,1,8,6,840,250,1080,250);//vision
	//视觉锁定框
//	ui_data_packed_rectangle(&gim_status_init[5],vis_draw,1,1,0,2,959,539,961,541);//
	//刻度线
	ui_data_packed_line(&scale_line[0],scale1,1,0,5,2,1780,760,1820,760);//20°
	ui_data_packed_line(&scale_line[1],scale2,1,0,5,2,1780,740,1820,740);//10°
	ui_data_packed_line(&scale_line[2],scale3,1,0,5,2,1780,720,1820,720);//0°
  ui_data_packed_line(&scale_line[3],scale4,1,0,5,2,1780,660,1820,660);//-30°
	ui_data_packed_line(&scale_line[4],scale5,1,0,5,2,1780,600,1820,600);//-60°
	//视觉相机范围框
//	ui_data_packed_line(&scale_line[0],scale1,1,0,5,2,700,700,1000,700);//20°
//	ui_data_packed_line(&scale_line[1],scale2,1,0,5,2,700,700,1000,700);//10°
//	ui_data_packed_line(&scale_line[2],scale3,1,0,5,2,700,1000,1000,1000);//0°
//  ui_data_packed_line(&scale_line[3],scale4,1,0,5,2,1000,700,1000,1000);//-30°
	//刻度文本打包
	ui_data_packed_int(&scale_num[0],sc_num1,1,1,5,10,2,1755,763,20);
	ui_data_packed_int(&scale_num[1],sc_num2,1,1,5,10,2,1755,743,10);
	ui_data_packed_int(&scale_num[2],sc_num3,1,1,5,10,2,1758,723,0);
	ui_data_packed_int(&scale_num[3],sc_num4,1,1,5,10,2,1752,663,-30);
	ui_data_packed_int(&scale_num[4],sc_num5,1,1,5,10,2,1752,603,-60);
}
//ui动态数据打包
void ui_dynamic_data_pack(void)
{
	//云台模式
	switch(ctrl_mode)
	{
		case REMOTER_MODE:
		{
			ui_data_packed_line(&gimbal_status[0],gim_mode3,2,0,5,8,840,227,920,227);//云台状态滑块
			ui_data_packed_line(&gimbal_status[1],vis_mode,2,1,8,6,840,250,1080,250);//vision状态机
			ui_data_packed_rectangle(&gimbal_status[2],vis_draw,2,1,0,2,959,539,961,541);//视觉方框959,539,961,541
		}break;
		
		case PROTECT_MODE:
		{
			ui_data_packed_line(&gimbal_status[0],gim_mode3,2,0,5,8,920,227,1000,227);//云台状态滑块
			ui_data_packed_line(&gimbal_status[1],vis_mode,2,1,8,6,840,250,1080,250);//vision状态机
			ui_data_packed_rectangle(&gimbal_status[2],vis_draw,2,1,0,2,959,539,961,541);//视觉方框
		}break;
		
		case KEYBOARD_MODE:
		{
			ui_data_packed_line(&gimbal_status[0],gim_mode3,2,0,5,8,1000,227,1080,227);//云台状态滑块
            ui_data_packed_line(&gimbal_status[1],vis_mode,2,1,8,6,840,250,1080,250);//vision状态机
			ui_data_packed_rectangle(&gimbal_status[2],vis_draw,2,1,0,2,959,539,961,541);//视觉方框
		}break;
		case VISION_MODE:
		{
			ui_data_packed_line(&gimbal_status[0],gim_mode3,0,0,5,8,1000,257,1080,257);//云台状态滑块
			if(vision.aim_status==AIMING)
			{
				ui_data_packed_line(&gimbal_status[1],vis_mode,2,1,0,6,840,250,1080,250);//vision状态机
			}
			else
			{
				ui_data_packed_line(&gimbal_status[1],vis_mode,2,1,2,6,840,250,1080,250);//vision状态机
			}
			
			
			//视觉坐标
			if((vir_x-5<0)||vir_x+5>1920)
			{
				if(vir_x-5<0) vir_x=5;
				else vir_x=1915;
			}
			if((vir_y-5<0)||vir_y+5>1920)
			{
				if(vir_y-5<0) vir_y=5;
				else vir_y=1915;
			}
			ui_data_packed_rectangle(&gimbal_status[2],vis_draw,2,1,0,2,vir_x-5,vir_y-5,vir_x+5,vir_y+5);
			
		}
		default:break;
	}
	//发射机构模式
	switch(shoot.firc_mode)
	{
		case FIRC_MODE_STOP:
		{
			ui_data_packed_arc(&gimbal_status[3],fric_mode,2,0,8,2,220,140,960,540,395,395);//Fric_R
	       
		}break;
		
		case FIRC_MODE_RUN:
		{
			if((rc.mouse.l || rc.ch5 >= 500)&&shoot.stir_mode!=STIR_MODE_PROTECT)
			{
				if(SHOOT_PERIOD_CHANGE_FLAG)
				{
				ui_data_packed_arc(&gimbal_status[3],fric_mode,2,0,1,2,220,140,960,540,395,395);
				}
				else
				{					
				ui_data_packed_arc(&gimbal_status[3],fric_mode,2,0,0,2,220,140,960,540,395,395);
				}
				
	           
			}
			else
			{
				ui_data_packed_arc(&gimbal_status[3],fric_mode,2,0,2,2,220,140,960,540,395,395);//Fric_R
			}
		}break;
		default:break;
	}
	
	ui_data_packed_line(&gimbal_status[4],pit,2,0,6,4,1790,720-2*imu_data.pitch,1810,720-2*imu_data.pitch);//IMU_pit

//  ui_data_packed_int(&lock_flag_ui[1],lock_flag_ui1,1,1,5,10,2,1752,700,lock_flag)
	
}
//ui初始化数据包发送
void ui_init_data_send(void)
{
     static int8_t i=0;
	switch(i++)
	{
		case 0:ui_sendgraph(DRAW_IMAGINE7,(uint8_t *) &scale_line, CLIENT_DRAW_7_GRAPHS_CMD_ID);	
			break;
		case 4:ui_sendgraph(DRAW_IMAGINE7,(uint8_t *) background, CLIENT_DRAW_7_GRAPHS_CMD_ID);
		    break;
		case 8:ui_sendgraph(DRAW_IMAGINE7,(uint8_t *) gim_status_init, CLIENT_DRAW_7_GRAPHS_CMD_ID);
            break;
		case 12:
			ui_sendgraph(DRAW_IMAGINE7,(uint8_t *) scale_num, CLIENT_DRAW_7_GRAPHS_CMD_ID);
//		case 16:
//			ui_sendgraph(DRAW_IMAGINE7,(uint8_t *) camera_line, CLIENT_DRAW_7_GRAPHS_CMD_ID);
		default:if(i>16) i=0;
		     break;		
	}
}
