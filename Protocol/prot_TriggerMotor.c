/**
  * @file bsp_TriggerMotor.c
  * @version 1.2
  * @date  2023..14
	*
  * @brief 1.0��ƽƽ����Ĳ��̴���
	*        1.1�������˵���һ��reload
	*				 1.2�����˵������߼��������ǵ�Ƶ����
	*
  *	@author  ������Ӳ��д��
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
//PID�ṹ��
pid_t pid_trigger_ecd;
pid_t pid_trigger_spd;

//�����ǿ����˵����
uint8_t back_flag_tri = 0;
uint16_t back_flag;
uint16_t back_flag_cnt;
uint16_t last_back_flag;
uint16_t err_flag;
uint16_t err_time;
uint32_t last_tri_ecd;

//2025��5��15�� ��Ƶ�任��ر���
uint8_t SHOOT_PERIOD_CHANGE_CNT;
uint8_t SHOOT_PERIOD_CHANGE_FLAG;
/*********************************************************************/

trigger_t trigger;


uint8_t AMMO_STUCK_FLAG= 0;                 //������־λ 1Ϊ���� 0Ϊ����
uint8_t SINGLE_AMMO_STUCK_BACK_FLAG= 0;          //���ڵ��������˵�ֻ��һ�ε��߼��ж�
uint8_t STIR_SINGLE_SHOT_FLAG;
uint8_t STIR_SERIES_SHOT_FLAG;
uint16_t reload_ammo_cnt=0;	//װ�ӵ�����
//static uint8_t  shoot_enable = 1;    //����ʹ�ܱ�־
uint16_t test_Trigger=6800;//��˽�����ʱ����ٶ�
uint32_t SINGLE_STUCK_CNT		= 0;                   //������������
uint32_t SERIES_STUCK_CNT		= 0;                   //������������
 
 
/**
  * @name     TriggerMotor_init
  * @brief    2006��ʼ��
  * @param    None
  * @retval   None
  * @note     �ٶȻ� 2  0.003  20 �����ܺ�
						
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
    shoot.pid.trigger_spd_ref = pid_trigger_ecd.pos_out;  //λ�û�

    shoot.pid.trigger_spd_fdb = trigger_motor.speed_rpm;
    pid_calc(&pid_trigger_spd, shoot.pid.trigger_spd_fdb, shoot.pid.trigger_spd_ref);
    shoot.current = pid_trigger_spd.pos_out;  //�ٶȻ�

    trigger_motor.tx_current = shoot.current;
}


///*���������жϺ�����single_shoot_enableͨ��������жϡ�����ģʽ�£���ecd_err����4����λ�ã�����Ϊ������ ��4�ε�������ͨ���������̶�ûת����0.5��.ע�⣬�ú����ڵ���ecd+=��ִ�У����Ըú����ڵ�ecd_err�������ٴ���1����λ�ã�����ض�����*/
//static void single_ammo_stuck_judge(void) 
//{
////					if(ABS(shoot.pid.trigger_ecd_error) > 0.5f * TRIGGER_MOTOR_ECD)
////					{
////							SINGLE_STUCK_CNT ++ ;
////							if(SINGLE_STUCK_CNT >=4)//3�β����̲���λ������Ϊ����
////							{
////								AMMO_STUCK_FLAG = 1;//��λ���������˵���־λ�����ں�����һ��
////								SINGLE_AMMO_STUCK_BACK_FLAG=1;
////							}
////					}
//	          if(ABS(shoot.pid.trigger_ecd_error) > 4.0f * TRIGGER_MOTOR_ECD)
//					{
////							SINGLE_STUCK_CNT ++ ;
////							if(SINGLE_STUCK_CNT >=4)//3�β����̲���λ������Ϊ����
////							{
//								AMMO_STUCK_FLAG = 1;//��λ���������˵���־λ�����ں�����һ��
//								SINGLE_AMMO_STUCK_BACK_FLAG=1;
////							}
//					}
//					else
//					{
//						AMMO_STUCK_FLAG = 0;
//					}	
//}

///*������λ���� 2025��4��21�ռǵý����ͼ����·�����ƴ˴���*/
//static uint8_t single_shoot_reset(void) 
//{
////    return (
////							(   (rc.mouse.l)==0  &&  ctrl_mode==VISION_MODE  &&  vsn_drive.shoot_enable )    //�Ӿ�ģʽ�£��ɿ�������&�Ӿ��������䣬�Ÿ�λ
////							|| (ABS(rc.ch5) < 10 && ctrl_mode == REMOTER_MODE)                             //ң��ģʽ�£������ɿ����ص����ӽ���ԭλ���Ÿ�λ
////							|| ((rc.mouse.l) == 0 && ctrl_mode == KEYBOARD_MODE)                       //����ģʽ�£��ɿ����������Ÿ�λ
////           );
//}

//������λenable����ң�ط��͵�������
uint8_t single_shoot_enable(void) {
	 #if NEW_RC//2025��7��3�� ��ң������û�еڶ������ˣ����Բ���дң�ش����ĵ����ˡ���������Ӿ�Ҫ�����������ôд
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
//        && (shoot.frequency_cnt  >= TRIGGER_PERIOD)  /* ��Ƶ���� */
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
        && (shoot.frequency_cnt  >= TRIGGER_PERIOD)  /* ��Ƶ���� */
    );
//	}
//    #endif
}



/**
  * @name     TriggerMotor_control
  * @brief    ��Ƶ����
  * @param    None
  * @retval   None
  * @note     2025��3��31�ճ�����
						
  */
void TriggerMotor_control(void)
{
	if(RC_FAIL_KEYBOARD_FLAG==1)
	{
//		shoot.stir_mode=STIR_MODE_SERIES;
	}
	
      switch (shoot.stir_mode) {
        case STIR_MODE_PROTECT: 
				{ //���̱���ģʽ�����ֹ��ԣ�����
            shoot.frequency_cnt = 0; //��ʱ������0�������ǰһ������ֹ
            shoot.trigger_period = 0;
						shoot.pid.trigger_ecd_ref = trigger_motor.total_ecd;//����ԭ��ͣ�£������鲦��
						shoot.pid.trigger_ecd_fdb = trigger_motor.total_ecd;            
					  pid_trigger_spd.iout  = 0;
				trigger_motor.tx_current = 0;
            break;
        }

        case STIR_MODE_SINGLE: 
					{ 
					 shoot.pid.trigger_ecd_error = shoot.pid.trigger_ecd_ref - shoot.pid.trigger_ecd_fdb;
//           if(!AMMO_STUCK_FLAG)//�ǿ�����־λ ������
//				   { 					
//							/*ִ�е����߼�*/
//							if (single_shoot_reset())
//							{//���η����ȴ���λ
//									trigger.trigger_enable = T_ENABLE;
//							}
//							if (single_shoot_enable())
//							{ //�Ѹ�λ���ҵ��������ж�ͨ������ecd+=��ͬʱ�ж��Ƿ񿨵�
//									trigger.trigger_enable = T_DISABLE;//�Ѿ�ִ��ecd+=����Ҫ�ٴθ�λ�������ٴ�ecd+=
//								  shoot.pid.trigger_ecd_ref += TRIGGER_MOTOR_ECD;
//								  single_ammo_stuck_judge();//ִ�п����ж�
//							}
//				   }
//					else//��AMMO_STUCK_FLAGΪ1����Ϊ������ִ���˵�����
//					{
//						if(SINGLE_AMMO_STUCK_BACK_FLAG==1)//��Ϊ���else����Ҫ�ȴ���һ��������λ���������ecd-=Ҫ����������ִֻ֤��һ��
//						{
//							shoot.pid.trigger_ecd_ref -= TRIGGER_MOTOR_ECD; //��һ��
//							SINGLE_AMMO_STUCK_BACK_FLAG=0;//ecd-=һ�������㵥�������˵���־λ����ֹ���ecd-=	
//						}
//						
//						
//						if(rc.mouse.l || rc.ch5 > 500)//ecd-=��AMMO_STUCK_FLAG��ȻΪ1����Ȼ�����else����ʱ����������󵥷�����ȴ�PID���������ecd-=����CNT��ģ���ʱ��
//						{
//							SINGLE_STUCK_CNT++;
//							if(SINGLE_STUCK_CNT>=1000)//ecd_ref��һ��֮�󣬱����ǻ���Ҫ�ж��ܲ����˵�λ�ģ���û�������֪����1����������0.5����������0.1����λ�þ͹��ˣ� ����һ���˲���λҲ�������ģ��ʲ��жϣ�ֻ��һ��CNT��������ʱ���Ƽ��ι��أ� ��֪����������1000�������1��
//							{
//								SINGLE_STUCK_CNT = 0;//���㣬��һ�ο�������
//								AMMO_STUCK_FLAG=0;
//							}
//						}
//					}
					TriggerMotor_pidcal();
            break;
        }
				
        case STIR_MODE_SERIES: 
					{ //��������ģʽ��������ǹ����������Ӧ
            shoot.pid.trigger_ecd_error = shoot.pid.trigger_ecd_ref - shoot.pid.trigger_ecd_fdb;
						
					if(!back_flag_tri)//��������ˣ��Ͳ��ٽ��շ���ָ�ר������˵�������
				  {
//						if(remote_online.NEW_OLD_RC_FLAG == 1)
//						{
////						#if NEW_RC
//						if( (new_rc.mouse_left || new_rc.trigger == 1) )
//						{					
//							  /*2025��5��15�� �Ӹ���Ƶ�仯���� ��̨�ֿ���ͨ����סshift�������Ƶ*/
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
//						/*���ϴ��룬����ʱ��ֱ��ע��*/			
//									shoot.frequency_cnt++;
//							  /*��Ƶ�任�����������ش��뻹�У�������CNT���壬init����CNT����*/
//									if (series_shoot_enable() && back_flag_tri ==0) 
//									{ 
//										STIR_SERIES_SHOT_FLAG++;
//										shoot.frequency_cnt = 0;
//										shoot.pid.trigger_ecd_ref += TRIGGER_MOTOR_ECD;/* ��һ���ӵ� */							
//									}
//						}
//						else
//						{
//							shoot.pid.trigger_ecd_ref = trigger_motor.total_ecd;//����ԭ��ͣ�£������鲦��
//							shoot.pid.trigger_ecd_fdb = trigger_motor.total_ecd;   
//						}
////						
////					#else
//					}
//						else if(remote_online.NEW_OLD_RC_FLAG == 0)
//						{
							if( (rc.mouse.l || rc.ch5 >= 500) )
						{					
							  /*2025��5��15�� �Ӹ���Ƶ�仯���� ��̨�ֿ���ͨ����סshift�������Ƶ*/
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
						
						/*���ϴ��룬����ʱ��ֱ��ע��*/			
									shoot.frequency_cnt++;
							  /*��Ƶ�任�����������ش��뻹�У�������CNT���壬init����CNT����*/
									if (series_shoot_enable() && back_flag_tri ==0) 
									{ 
										STIR_SERIES_SHOT_FLAG++;
										shoot.frequency_cnt = 0;
										shoot.pid.trigger_ecd_ref += TRIGGER_MOTOR_ECD;/* ��һ���ӵ� */							
									}
						}
						else
						{
							shoot.pid.trigger_ecd_ref = trigger_motor.total_ecd;//����ԭ��ͣ�£������鲦��
							shoot.pid.trigger_ecd_fdb = trigger_motor.total_ecd;   
						}
						
//						#endif
//					}//else if ������
				 }

            /* �ȿ����жϣ��������ת������������ + ת�ٻή��*/
						if(ABS((int)trigger_motor.rx_current)>10000 && ABS(trigger_motor.speed_rpm)<500)//
							{
								back_flag_cnt++;
								trigger_motor.tx_current = 0;
							}
							
						/*������������������˵�*/
             if(back_flag_cnt > 200)
            {          
                shoot.pid.trigger_ecd_ref = trigger_motor.total_ecd;//���뵱ǰ����λ�ã����̵�λ�û�һ�㶼�����ϣ�������Զ������Ҫ���¶��룩
                last_tri_ecd = trigger_motor.total_ecd;//��¼��ǰ����λ�ã����������˳��˵����ж�
                back_flag_tri = 1;//��λ�˵���־λ ���沦���˵���
							  back_flag = 1;//��λһ��back_flag������ֻ-=һ��
								back_flag_cnt = 0;//���������������ֹ�ظ������if
            }   
						
						//ÿ�ο�����ֻ-=һ��ref
						if(back_flag==1)
						{
							  shoot.pid.trigger_ecd_ref -= 2.0f * TRIGGER_MOTOR_ECD;//��������λ��
							  back_flag = 0;//�������ֹ�ظ�-=ref
						}
						
						/*�ȴ��˵�ִ�н���*/
						if( back_flag_tri == 1 )//�˵���־λ ��������ִ���˵�
						{
							 err_time += SHOOT_PERIOD;//�˵�ִ�м�ʱ ��ʱ����ֱ���˳��˵������ٵȲ��̵�λ���˵�ʱ������Ȼ�ܴ󣬲��̲���λҲ������
							
							//�˵����������ݣ���ת��λ��2.0������ʱ��ִ��200�η�������
							if((err_time >= 200) || (trigger_motor.total_ecd == (last_tri_ecd - 2.0*TRIGGER_MOTOR_ECD)) )
							{
								  err_time = 0;//����˵�ִ��ʱ�� ��ֹӰ����һ���˵�
								  back_flag_tri=0;//����˵���־λ 
							}
					  }
						
						/*�����˵��������*/
						
						TriggerMotor_pidcal();						
            break;
        }
				
        default: {break;}
    }
	dji_send_current(NULL, NULL, &trigger_motor, NULL);
//    osSignalSet(can_msg_send_task_t, TRIGGER_MOTOR_MSG_SEND);
}

