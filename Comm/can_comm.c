/**
  ******************************************************************************
  * @file    bsp.c
  * @author  ZZY 1950828677@qq.com
  * @brief   对所有bsp层库文件的总调用，包括：初始化；发送；接收
  * @date    2025-10
  * @version 1.0
  * @note 
	
 * -----------------------------------���˹������-----------------------------------
 * ���������Ķ����ʵ��������bsp_can���û����м���������������
 * count�����can���˹�������������10����������ȥbsp_can.h��bsp_can_filt_tbl_t��data����ĳ��ȣ�
 * ������һ������ռdata������3��λ�ã�ע���ʽ��type;id_high;id_low ����Ϊu16���ͣ�������
 * Type����Щ����ѡ��
            * FLT_RANGE = 0x0000U
            * FLT_DUAL  = 0x0001U
            * FLT_MASK  = 0x0002U

*/

/* Includes ------------------------------------------------------------------*/
#include "can_comm.h"
#include "prot_rm_motor.h"
#include "prot_dm_motor.h"
#include "prot_T_imu.h"
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
//moto_measure_t moto_test;
uint32_t trigger_can_receive_cnt=0;
uint32_t fric_can_recive_cnt = 0;
uint32_t yaw_can_receive_cnt = 0;
/*-----------------------------���˹��������-----------------------------------*/

/* Private type --------------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/*-----------------------------CAN��ʼ����ʼ-----------------------------------*/
/**
* @brief  ��ʼ��CAN
* @param  bsp_user_can_t BSP���װ��CAN�˿�
* @retval None
*/
void comm_can_init(void)
{
	  /*-----------�û��ڴ��޸Ĺ��������ݣ����10�����򣬵�������bsp�ļ���ģ�--------*/
    bsp_can1_filt_tbl.count = 2;//CAN1һ�����˶��������˹���
	  bsp_can1_filt_tbl.data[0]=FLT_DUAL;
	  bsp_can1_filt_tbl.data[1]=0x206&0x7ff;
	  bsp_can1_filt_tbl.data[2]=0x207&0x7ff;
		bsp_can1_filt_tbl.data[3]=FLT_DUAL;
		bsp_can1_filt_tbl.data[4]=0x017&0x7ff;
		bsp_can1_filt_tbl.data[5]=0x018&0x7ff;
	bsp_can2_filt_tbl.count = 2;//CAN2һ�����˶��������˹���
	  bsp_can2_filt_tbl.data[0]=FLT_DUAL;
	  bsp_can2_filt_tbl.data[1]=0x206&0x7ff;
	  bsp_can2_filt_tbl.data[2]=0x007&0x7ff;
		bsp_can2_filt_tbl.data[3]=FLT_DUAL;
		bsp_can2_filt_tbl.data[4]=0x001&0x7ff;
		bsp_can2_filt_tbl.data[5]=0x007&0x7ff;
    bsp_can3_filt_tbl.count = 1;//CAN3һ�����˶��������˹���
	  bsp_can3_filt_tbl.data[0]=FLT_DUAL;
	  bsp_can3_filt_tbl.data[1]=0x206&0x7ff;
	  bsp_can3_filt_tbl.data[2]=0x207&0x7ff;
	

	  /*�ٵ��ú�������ʼ��*/
    bsp_can_init(&bsp_user_can1,User_CAN1_RxCpltCallback);
    bsp_can_init(&bsp_user_can2,User_CAN2_RxCpltCallback);
    bsp_can_init(&bsp_user_can3,User_CAN3_RxCpltCallback);
	dji_motor_init(&fric_motor[0], DJI_3508_MOTOR,CAN_CHANNEL_1, 0x201, 1.0f);
    dji_motor_init(&fric_motor[1], DJI_3508_MOTOR,CAN_CHANNEL_1, 0x202, 1.0f);
		
    dji_motor_init(&trigger_motor, DJI_2006_MOTOR,CAN_CHANNEL_1, 0x207, 82.8);

    dji_motor_init(&yaw_motor, DJI_6020_MOTOR,CAN_CHANNEL_2, 0x206, 1.0f);
	dm_motor_init(&pitch_motor,DM4310,CAN_CHANNEL_2,DM_PITCH_RECEIVE_ID,DM_PITCH_SEND_ID,DM_4310_RATIO,DM_PITCH_OFFSET);
	dm_motor_init(&dm3507_motor,DM3507,CAN_CHANNEL_2,DM_3507_RECEIVE_ID,DM_3507_SEND_ID,DM_3507_RATIO,DM_3507_OFFEST);
}
    

/*-----------------------------CAN��ʼ����ʼ-----------------------------------*/


/**
* @brief  Callback function in CAN Interrupt
* @param  None.
* @return None.
*/
void User_CAN1_RxCpltCallback(CAN_RxBuffer_t* buf)
{
//    switch (CAN_RxBuffer_t->rxHeader->Identifier)
//    {
//    case constant expression:{

//    }break;
//    
//    default:{break;}
//    }
	
	switch(bsp_user_can1.CAN_RxBuffer.rxHeader.Identifier)
			{
				case CAN_3508_FRIC_L_ID :
				case CAN_3508_FRIC_R_ID :
				{
					fric_can_recive_cnt++;
					static uint8_t i;
					i = bsp_user_can1.CAN_RxBuffer.rxHeader.Identifier - CAN_3508_FRIC_L_ID ;
					dji_motor_get_single_data(&fric_motor[i],bsp_user_can1.CAN_RxBuffer.rx_data);
					switch (i)
					{
//						case 0: wdg_user_set_bit(WDG_BIT_BSP_FRIC_MOTO1); break;
//						case 1: wdg_user_set_bit(WDG_BIT_BSP_FRIC_MOTO2); break;
						default: break;
					}
					break;
				}
				case CAN_YAW_MOTOR_ID:
				{
					
					dji_motor_get_single_data(&yaw_motor,bsp_user_can1.CAN_RxBuffer.rx_data);
//					wdg_user_set_bit(WDG_BIT_BSP_GIMBAL_YAW);
			break;
				}
				case TIMU_PALSTANCE_ID:
				{
					T_imu_calcu(bsp_user_can1.CAN_RxBuffer.rxHeader.Identifier, bsp_user_can1.CAN_RxBuffer.rx_data);
					break;
				}
				case TIMU_ANGLE_ID:
				{
					T_imu_calcu(bsp_user_can1.CAN_RxBuffer.rxHeader.Identifier,bsp_user_can1.CAN_RxBuffer.rx_data);
//					wdg_user_set_bit(WDG_BIT_BSP_IMU_ANGLE);
					break;
				}
				case CAN_TRIGGER_MOTOR1_ID :
				{
					trigger_can_receive_cnt++;
					dji_motor_get_single_data(&trigger_motor, bsp_user_can1.CAN_RxBuffer.rx_data);   
					break;
				}
				default:break;
			}
}

void User_CAN2_RxCpltCallback(CAN_RxBuffer_t* buf)
{
//    switch (CAN_RxBuffer_t->rxHeader->Identifier)
//    {
//    case constant expression:{

//    }break;
//    
//    default:{break;}
//    }
	switch(bsp_user_can2.CAN_RxBuffer.rxHeader.Identifier)
			{
				case DM_3507_RECEIVE_ID:
				{
					dm_get_single_data(&dm3507_motor,bsp_user_can2.CAN_RxBuffer.rx_data);
					break;
				}
				case CAN_YAW_MOTOR_ID:
				{
					
					dji_motor_get_single_data(&yaw_motor,bsp_user_can2.CAN_RxBuffer.rx_data);
//					wdg_user_set_bit(WDG_BIT_BSP_GIMBAL_YAW);
					break;
				}
				case CAN_J4310_MOTOR_ID:
				{
					dm_get_single_data(&pitch_motor,bsp_user_can2.CAN_RxBuffer.rx_data);
//					encoder_J4310_get(&motor_dm4310_pitch,rx_fifo0_data);
					break;
				}
				case TIMU_PALSTANCE_ID:
				{
					T_imu_calcu(bsp_user_can2.CAN_RxBuffer.rxHeader.Identifier, bsp_user_can2.CAN_RxBuffer.rx_data);
					break;
				}
				case TIMU_ANGLE_ID:
				{
					T_imu_calcu(bsp_user_can2.CAN_RxBuffer.rxHeader.Identifier,bsp_user_can2.CAN_RxBuffer.rx_data);
//					wdg_user_set_bit(WDG_BIT_BSP_IMU_ANGLE);
					break;
				}
				default:break;
			}

}

void User_CAN3_RxCpltCallback(CAN_RxBuffer_t* buf)
{
//    switch (CAN_RxBuffer_t->rxHeader->Identifier)
//    {
//    case constant expression:{

//    }break;
//    
//    default:{break;}
//    }
}

/**
* @brief  CAN��������
* @param  None.
* @return None.
*/
void Task_CAN1Transmit(void const *arg)
{
  /* Cache for Task */
 static uint8_t  can1_free_mailbox;
  /* Pre-Load for task */
   uint32_t wake_time = osKernelSysTick();
  /* Infinite loop */
  
  for(;;)
  {
	  dji_send_current(&fric_motor[0],&fric_motor[1],NULL,NULL);
	  osDelayUntil(&wake_time, 1);
    /* CAN1 Send Port */
//    if(xQueueReceive(CAN1_TxPort,&CAN_TxMsg,portMAX_DELAY) == pdPASS)
//    {
//      do{
//        free_can_mailbox = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
//      }while(free_can_mailbox == 0);
//      bsp_can_std_send(&bsp_user_can1,CAN_TxMsg.ID,CAN_TxMsg.Data);
//    }
  }
}

void Task_CAN2Transmit(void const *arg)
{
  /* Cache for Task */
//  uint8_t free_can_mailbox;
  /* Pre-Load for task */
   uint32_t wake_time = osKernelSysTick();
  /* Infinite loop */
  
  for(;;)
  {
	  dji_send_current(&pit_motor,&yaw_motor,&trigger_motor ,NULL);
	  dm_mit_control(&pitch_motor);
	  dm_mit_control(&dm3507_motor);
	  osDelayUntil(&wake_time, 1);
    /* CAN2 Send Port */
//    if(xQueueReceive(CAN2_TxPort,&CAN_TxMsg,portMAX_DELAY) == pdPASS)
//    {
//      do{
//        free_can_mailbox = HAL_CAN_GetTxMailboxesFreeLevel(&hcan2);
//      }while(free_can_mailbox == 0);
//      bsp_can_std_send(&bsp_user_can2,CAN_TxMsg.ID,CAN_TxMsg.Data);
//    }
  }
}


void Task_CAN3Transmit(void const *arg)
{
  /* Cache for Task */
//  uint8_t free_can_mailbox;

  /* Pre-Load for task */
   uint32_t wake_time = osKernelSysTick();
  /* Infinite loop */
  
  for(;;)
  {
	  osDelayUntil(&wake_time, 1);
    /* CAN2 Send Port */
//    if(xQueueReceive(CAN2_TxPort,&CAN_TxMsg,portMAX_DELAY) == pdPASS)
//    {
//      do{
//        free_can_mailbox = HAL_CAN_GetTxMailboxesFreeLevel(&hcan2);
//      }while(free_can_mailbox == 0);
//      bsp_can_std_send(&bsp_user_can3,CAN_TxMsg.ID,CAN_TxMsg.Data);
//    }
  }
}

//void encoder_data_handler(moto_measure_t *ptr, FDCAN_HandleTypeDef *hfdcan, uint8_t *CAN_Rx_data)
//{
//    // ת��ת��
//    ptr->speed_rpm = (int16_t)(CAN_Rx_data[2] << 8 | CAN_Rx_data[3]);
//    ptr->given_current = (int16_t)(CAN_Rx_data[4] << 8 | CAN_Rx_data[5]);

//    // ��е�Ƕ�
//    ptr->last_ecd = ptr->ecd;
//    ptr->ecd = (uint16_t)(CAN_Rx_data[0] << 8 | CAN_Rx_data[1]);

//    // ��Կ�����ĽǶ�
//    if (ptr->ecd - ptr->last_ecd > 4096)
//        ptr->round_cnt--;
//    else if (ptr->ecd - ptr->last_ecd < -4096)
//        ptr->round_cnt++;

//    ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
//}
