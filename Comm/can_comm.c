/**
  ******************************************************************************
  * @file    bsp.c
  * @author  ZZY 1950828677@qq.com
  * @brief   对所有bsp层库文件的总调�?，包�?：初始化；发送；接收
  * @date    2025-10
  * @version 1.0
  * @note 
	
 * -----------------------------------���˹������?-----------------------------------
 * ���������Ķ����ʵ��������bsp_can���û����м���������������
 * count�����can���˹�������������10����������ȥbsp_can.h��bsp_can_filt_tbl_t��data����ĳ��ȣ�?
 * ������һ������ռdata������3��λ�ã�ע���ʽ��type;id_high;id_low ����Ϊu16���ͣ�������
 * Type����Щ����ѡ��
            * FLT_RANGE = 0x0000U
            * FLT_DUAL  = 0x0001U
            * FLT_MASK  = 0x0002U

*/

/* Includes ------------------------------------------------------------------*/
#include "can_comm.h"

/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
//moto_measure_t moto_test;
uint32_t trigger_can_receive_cnt=0;
uint32_t fric_can_recive_cnt = 0;
uint32_t yaw_can_receive_cnt = 0;
/*-----------------------------���˹��������?-----------------------------------*/

/* Private type --------------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/
extern QueueHandle_t CAN1_Tx_queue;
extern QueueHandle_t CAN2_Tx_queue;
extern QueueHandle_t CAN3_Tx_queue;
/* function prototypes -------------------------------------------------------*/

/*-----------------------------CAN��ʼ����ʼ-----------------------------------*/
/**
* @brief  ��ʼ��CAN
* @param  bsp_user_can_t BSP���װ��CAN�˿�
* @retval None
*/
void comm_can_init(void)
{
	  /*-----------�û��ڴ��޸Ĺ��������ݣ����?10�����򣬵�������bsp�ļ���ģ�?--------*/
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
	dm_motor_init(&pitch_motor,DM4310,PROT_CAN_CHANNEL_2,DM_PITCH_RECEIVE_ID,DM_PITCH_SEND_ID,DM_4310_RATIO,DM_PITCH_OFFSET);
	dm_motor_init(&dm3507_motor,DM3507,PROT_CAN_CHANNEL_2,DM_3507_RECEIVE_ID,DM_3507_SEND_ID,DM_3507_RATIO,DM_3507_OFFEST);
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



/*-----------------------------CAN���Ͳ��֣����з��ͽӿ�����н��շ�������? CAN Send function (queue send interface)-----------------------------------*/

/**
* @brief  
* @param  None.
* @return None.
*/
void Prot_CAN1_Send_Frame(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    CanFrame_t msg = {0};

    msg.ID   = id;
    msg.DLC  = (dlc > 8) ? 8 : dlc;

    if (data != NULL)
    {
        memcpy(msg.Data, data, msg.DLC);
    }

    BaseType_t ret = xQueueSend(CAN1_Tx_queue, &msg, 0);

    if (ret != pdPASS)
    {
        // ��ѡ����֡����
    }
}

/**
* @brief  
* @param  None.
* @return None.
*/void Prot_CAN2_Send_Frame(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    CanFrame_t msg = {0};

    msg.ID   = id;
    msg.DLC  = (dlc > 8) ? 8 : dlc;

    if (data != NULL)
    {
        memcpy(msg.Data, data, msg.DLC);
    }

    BaseType_t ret = xQueueSend(CAN2_Tx_queue, &msg, 0);

    if (ret != pdPASS)
    {
        // ѡ֡
    }
}

/**
* @brief  
* @param  None.
* @return None.
*/
void Prot_CAN3_Send_Frame(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    CanFrame_t msg = {0};

    msg.ID   = id;
    msg.DLC  = (dlc > 8) ? 8 : dlc;

    if (data != NULL)
    {
        memcpy(msg.Data, data, msg.DLC);
    }

    BaseType_t ret = xQueueSend(CAN3_Tx_queue, &msg, 0);

    if (ret != pdPASS)
    {
        // ѡ֡
    }
}




/*-----------------------------CAN�������� CAN Send task (queue receive)-----------------------------------*/

/**
* @brief  CAN��������
* @param  None.
* @return None.
*/
void Task_CAN1Transmit(void const *arg)
{
  /* Cache for Task */
//  uint8_t  can1_free_mailbox;
  CanFrame_t msg;
  /* Pre-Load for task */
  uint32_t wake_time = osKernelSysTick();
  /* Infinite loop */
    for (;;)
    {
        if (xQueueReceive(CAN1_Tx_queue, &msg, portMAX_DELAY) == pdPASS)
        {
            uint32_t timeout = 100;  // 最大等待 100ms（可调）
            while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0 && timeout > 0)
            {
                vTaskDelay(pdMS_TO_TICKS(1));  // 每1ms检查一次，让出CPU
                timeout--;
            }

            if (timeout > 0)
            {
                // 有空闲邮箱，发送
                bsp_can_std_transmit(CAN_CHANNEL_1, msg.ID, msg.Data);
            }
            else
            {
                // 超时：丢帧或日志
                // can_tx_timeout_cnt++;
            }

        }
    }
}

void Task_CAN2Transmit(void const *arg)
{

  /* Cache for Task */
//  uint8_t  can2_free_mailbox;
  CanFrame_t msg;
  /* Pre-Load for task */
  uint32_t wake_time = osKernelSysTick();
  /* Infinite loop */
    for (;;)
    {
        if (xQueueReceive(CAN2_Tx_queue, &msg, portMAX_DELAY) == pdPASS)
        {
            uint32_t timeout = 100;  // 最大等待 100ms（可调）
            while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0 && timeout > 0)
            {
                vTaskDelay(pdMS_TO_TICKS(1));  // 每1ms检查一次，让出CPU
                timeout--;
            }

            if (timeout > 0)
            {
                // 有空闲邮箱，发送
                bsp_can_std_transmit(CAN_CHANNEL_2, msg.ID, msg.Data);
            }
            else
            {
                // 超时：丢帧或日志
                // can_tx_timeout_cnt++;
            }
        }
    }
}


void Task_CAN3Transmit(void const *arg)
{
  /* Cache for Task */
//  uint8_t  can3_free_mailbox;
  CanFrame_t msg;
  /* Pre-Load for task */
  uint32_t wake_time = osKernelSysTick();
  /* Infinite loop */
    for (;;)
    
    {
        if (xQueueReceive(CAN3_Tx_queue, &msg, portMAX_DELAY) == pdPASS)
        {
            uint32_t timeout = 100;  // 最大等待 100ms（可调）
            while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan3) == 0 && timeout > 0)
            {
                vTaskDelay(pdMS_TO_TICKS(1));  // 每1ms检查一次，让出CPU
                timeout--;
            }

            if (timeout > 0)
            {
                // 有空闲邮箱，发送
                bsp_can_std_transmit(CAN_CHANNEL_3, msg.ID, msg.Data);
            }
            else
            {
                // 超时：丢帧或日志
                // can_tx_timeout_cnt++;
            }
        }
    }
}
