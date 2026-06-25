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
#include "dbg_print.h"
#include "config.h"

/* Private define ------------------------------------------------------------*/
#define TX_RETRY_MAX       5      // CAN等邮箱：总等待 ≈5ms
#define TX_RETRY_DELAY_MS  1      // CAN等邮箱：每次检查间隔 1ms
/* Private variables ---------------------------------------------------------*/
//moto_measure_t moto_test;
uint32_t trigger_can_receive_cnt=0;
uint32_t fric_can_recive_cnt = 0;
uint32_t yaw_can_receive_cnt = 0;

/* 外部电机变量 */
extern dji_motor_t chassis_2006[4];
extern dji_motor_t arm_yaw_motor;
extern dji_motor_t trigger_motor;
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
    bsp_can1_filt_tbl.count = 5;
	  bsp_can1_filt_tbl.data[0]=FLT_DUAL;
	  bsp_can1_filt_tbl.data[1]=0x017&0x7ff;  // IMU 角速度
	  bsp_can1_filt_tbl.data[2]=0x018&0x7ff;  // IMU 角度
		bsp_can1_filt_tbl.data[3]=FLT_DUAL;
		bsp_can1_filt_tbl.data[4]=0x019&0x7ff;  // IMU roll
		bsp_can1_filt_tbl.data[5]=0x019&0x7ff;
		bsp_can1_filt_tbl.data[6]=FLT_DUAL;
		bsp_can1_filt_tbl.data[7]=0x201&0x7ff;  // 右后底盘
		bsp_can1_filt_tbl.data[8]=0x202&0x7ff;  // 左后底盘
		bsp_can1_filt_tbl.data[9]=FLT_DUAL;
		bsp_can1_filt_tbl.data[10]=0x203&0x7ff;  // 右前底盘
		bsp_can1_filt_tbl.data[11]=0x204&0x7ff;  // 左前底盘
		bsp_can1_filt_tbl.data[12]=FLT_DUAL;
		bsp_can1_filt_tbl.data[13]=0x207&0x7ff;
		bsp_can1_filt_tbl.data[14]=0x208&0x7ff;
	bsp_can2_filt_tbl.count = 1;
	  bsp_can2_filt_tbl.data[0]=FLT_DUAL;
	  bsp_can2_filt_tbl.data[1]=0x206&0x7ff;  // yaw 6020
	  bsp_can2_filt_tbl.data[2]=0x206&0x7ff;
    bsp_can3_filt_tbl.count = 2;
	  bsp_can3_filt_tbl.data[0]=FLT_DUAL;
	  bsp_can3_filt_tbl.data[1]=0x201&0x7ff;  // 右后底盘
	  bsp_can3_filt_tbl.data[2]=0x202&0x7ff;  // 左后底盘
		bsp_can3_filt_tbl.data[3]=FLT_DUAL;
		bsp_can3_filt_tbl.data[4]=0x203&0x7ff;  // 右前底盘
		bsp_can3_filt_tbl.data[5]=0x204&0x7ff;  // 左前底盘
	

	  /*�ٵ��ú�������ʼ��*/
    bsp_can_init(&bsp_user_can1,User_CAN1_RxCpltCallback);
    bsp_can_init(&bsp_user_can2,User_CAN2_RxCpltCallback);
    bsp_can_init(&bsp_user_can3,User_CAN3_RxCpltCallback);

    /*电机初始化，挪到StartDefaultTask了。因为这些属于“设备初始化”而不是can外设初始化  Motor init,moved to StartDefaultTask,because "motor" != can Peripherals */
	// dji_motor_init(&fric_motor[0], DJI_3508_MOTOR,CAN_CHANNEL_1, 0x201, 1.0f);
    // dji_motor_init(&fric_motor[1], DJI_3508_MOTOR,CAN_CHANNEL_1, 0x202, 1.0f);
		
    // dji_motor_init(&trigger_motor, DJI_2006_MOTOR,CAN_CHANNEL_1, 0x207, 82.8);

    // dji_motor_init(&yaw_motor, DJI_6020_MOTOR,CAN_CHANNEL_2, 0x206, 1.0f);
    /* 机械臂底座 6020 初始化 */
    dji_motor_init(&arm_yaw_motor, DJI_6020_MOTOR, CAN_CHANNEL_1, CAN_ARM_YAW_MOTOR_ID, 1.0f);
	// dm_motor_init(&pitch_motor,DM4310,PROT_CAN_CHANNEL_2,DM_PITCH_RECEIVE_ID,DM_PITCH_SEND_ID,DM_4310_RATIO,DM_PITCH_OFFSET);
    // dm_motor_init(&dm3507_motor,DM3507,PROT_CAN_CHANNEL_2,DM_3507_RECEIVE_ID,DM_3507_SEND_ID,DM_3507_RATIO,DM_3507_OFFEST);

    /* 底盘2006电机初始化（全在 CAN1） */
    dji_motor_init(&chassis_2006[0], DJI_2006_MOTOR, CAN_CHANNEL_1, CHA_LEFT_FRONT_2006, DJI_2006_ORIGINAL_REDUCTION_RATIO);
    dji_motor_init(&chassis_2006[1], DJI_2006_MOTOR, CAN_CHANNEL_1, CHA_LEFT_BACK_2006, DJI_2006_ORIGINAL_REDUCTION_RATIO);
    dji_motor_init(&chassis_2006[2], DJI_2006_MOTOR, CAN_CHANNEL_1, CHA_RIGHT_FRONT_2006, DJI_2006_ORIGINAL_REDUCTION_RATIO);
    dji_motor_init(&chassis_2006[3], DJI_2006_MOTOR, CAN_CHANNEL_1, CHA_RIGHT_BACK_2006, DJI_2006_ORIGINAL_REDUCTION_RATIO);
    dji_motor_init(&trigger_motor, DJI_2006_MOTOR, CAN_CHANNEL_1, 0x207, 82.8f);
}
    

/*-----------------------------CAN��ʼ����ʼ-----------------------------------*/


/**
* @brief  Callback function in CAN Interrupt
* @param  None.
* @return None.
*/
void User_CAN1_RxCpltCallback(CAN_RxBuffer_t* buf)
{
	
	switch(bsp_user_can1.CAN_RxBuffer.rxHeader.Identifier)
			{
				case TIMU_PALSTANCE_ID:
				{
					T_imu_calcu(bsp_user_can1.CAN_RxBuffer.rxHeader.Identifier, bsp_user_can1.CAN_RxBuffer.rx_data);
					break;
				}
				case TIMU_ANGLE_ID:
				{
					T_imu_calcu(bsp_user_can1.CAN_RxBuffer.rxHeader.Identifier,bsp_user_can1.CAN_RxBuffer.rx_data);
					break;
				}
			case TIMU_ROL_ID:
				{
					T_imu_calcu(bsp_user_can1.CAN_RxBuffer.rxHeader.Identifier, bsp_user_can1.CAN_RxBuffer.rx_data);
					break;
				}
			case CHA_LEFT_FRONT_2006:
			case CHA_LEFT_BACK_2006:
			case CHA_RIGHT_FRONT_2006:
			case CHA_RIGHT_BACK_2006:
				{
					uint32_t id = bsp_user_can1.CAN_RxBuffer.rxHeader.Identifier;
					uint8_t idx;
					if      (id == CHA_LEFT_FRONT_2006) idx = 0;
					else if (id == CHA_LEFT_BACK_2006)  idx = 1;
					else if (id == CHA_RIGHT_FRONT_2006) idx = 2;
					else if (id == CHA_RIGHT_BACK_2006)  idx = 3;
					else break;  /* 未知 CAN ID，不应到达 */
					dji_motor_get_single_data(&chassis_2006[idx], bsp_user_can1.CAN_RxBuffer.rx_data);
					break;
				}
			case 0x207:
			case 0x208:
				{
					uint32_t id = bsp_user_can1.CAN_RxBuffer.rxHeader.Identifier;
					if (id == 0x207)
						dji_motor_get_single_data(&trigger_motor, bsp_user_can1.CAN_RxBuffer.rx_data);
					else
						dji_motor_get_single_data(&arm_yaw_motor, bsp_user_can1.CAN_RxBuffer.rx_data);
					break;
				}
				default:break;
			}
}

void User_CAN2_RxCpltCallback(CAN_RxBuffer_t* buf)
{
	switch(bsp_user_can2.CAN_RxBuffer.rxHeader.Identifier)
			{
				case CAN_YAW_MOTOR_ID:
				{
					dji_motor_get_single_data(&yaw_motor, bsp_user_can2.CAN_RxBuffer.rx_data);
					break;
				}
				default:break;
			}

}

void User_CAN3_RxCpltCallback(CAN_RxBuffer_t* buf)
{
		switch(bsp_user_can3.CAN_RxBuffer.rxHeader.Identifier)
			{
			case CHA_LEFT_FRONT_2006:
			case CHA_LEFT_BACK_2006:
			case CHA_RIGHT_FRONT_2006:
			case CHA_RIGHT_BACK_2006:
				{
					uint32_t id = bsp_user_can3.CAN_RxBuffer.rxHeader.Identifier;
					uint8_t idx;
					if      (id == CHA_LEFT_FRONT_2006) idx = 0;
					else if (id == CHA_LEFT_BACK_2006)  idx = 1;
					else if (id == CHA_RIGHT_FRONT_2006) idx = 2;
					else if (id == CHA_RIGHT_BACK_2006)  idx = 3;
					else break;
					dji_motor_get_single_data(&chassis_2006[idx], bsp_user_can3.CAN_RxBuffer.rx_data);
					break;
				}
			default:{break;}
			}
}


/*-----------------------------底盘电机发送函数-----------------------------------*/
void chassis_can_send(void)
{
    /* 走标准dji_send_current/dji_motor_fill_data路径，跟arm_yaw_motor等其它电机一致：
       打包按各自can_id自动定位字节偏移，send_cnt由dji_motor_fill_data统一维护，
       4个电机can_id都<0x205，自动合并发送同一个0x200帧 */
    dji_send_current(&chassis_2006[0], &chassis_2006[1], &chassis_2006[2], &chassis_2006[3]);
}


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
#if DEBUG_TEXT_ENABLE
        dbg_print("[CAN1TX] queue full, dropped id=0x%03X\r\n", (unsigned int)id);
#endif
    }
}

/**
* @brief  
* @param  None.
* @return None.
*/
void Prot_CAN2_Send_Frame(uint32_t id, const uint8_t *data, uint8_t dlc)
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
            uint32_t retry = 0;
//          HAL_StatusTypeDef hal_ret = HAL_ERROR;  // 初始错误
            do {
                // 直接调用 BSP 非阻塞接口（你上面那个版本已去掉 while）
                bsp_can_state_e status = bsp_can_std_transmit(CAN_CHANNEL_1, msg.ID, msg.Data);

                if (status == BSP_CAN_OK) {
                    break;  // 成功发送
                }

                if (status == BSP_CAN_LINE_BUSY) {
                    // 邮箱满，等一小会儿再试
                    vTaskDelay(pdMS_TO_TICKS(TX_RETRY_DELAY_MS));
                    retry++;
                } else {
                    // 其他错误（ERROR/INVALID_PARAM 等），直接丢或日志
#if DEBUG_TEXT_ENABLE
                    dbg_print("[CAN1TX] bsp error id=0x%03X status=%d\r\n", (unsigned int)msg.ID, (int)status);
#endif
                    break;
                }
            } while (retry < TX_RETRY_MAX);

        /*------------如果实在是太忙了就不等了------------*/
            if (retry >= TX_RETRY_MAX) {
                // 超时丢帧（统计丢帧率，必要时报警或通知上层降频）
#if DEBUG_TEXT_ENABLE
                dbg_print("[CAN1TX] retry exhausted, dropped id=0x%03X\r\n", (unsigned int)msg.ID);
#endif
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
			
            uint32_t retry = 0;
//          HAL_StatusTypeDef hal_ret = HAL_ERROR;  // 初始错误
            do {
                // 直接调用 BSP 非阻塞接口（你上面那个版本已去掉 while）
                bsp_can_state_e status = bsp_can_std_transmit(CAN_CHANNEL_2, msg.ID, msg.Data);

                if (status == BSP_CAN_OK) {
                    break;  // 成功发送
                }

                if (status == BSP_CAN_LINE_BUSY) {
                    // 邮箱满，等一小会儿再试
                    vTaskDelay(pdMS_TO_TICKS(TX_RETRY_DELAY_MS));
                    retry++;
                } else {
                    // 其他错误（ERROR/INVALID_PARAM 等），直接丢或日志
                    // can_tx_error_cnt++;
                    break;
                }
            } while (retry < TX_RETRY_MAX);

        /*------------如果实在是太忙了就不等了------------*/
            if (retry >= TX_RETRY_MAX) {
                // 超时丢帧（统计丢帧率，必要时报警或通知上层降频）
                // can_tx_drop_cnt++;
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
            uint32_t retry = 0;
//          HAL_StatusTypeDef hal_ret = HAL_ERROR;  // 初始错误
            do {
                // 直接调用 BSP 非阻塞接口（你上面那个版本已去掉 while）
                bsp_can_state_e status = bsp_can_std_transmit(CAN_CHANNEL_3, msg.ID, msg.Data);

                if (status == BSP_CAN_OK) {
                    break;  // 成功发送
                }

                if (status == BSP_CAN_LINE_BUSY) {
                    // 邮箱满，等一小会儿再试
                    vTaskDelay(pdMS_TO_TICKS(TX_RETRY_DELAY_MS));
                    retry++;
                } else {
                    // 其他错误（ERROR/INVALID_PARAM 等），直接丢或日志
                    // can_tx_error_cnt++;
                    break;
                }
            } while (retry < TX_RETRY_MAX);

        /*------------如果实在是太忙了就不等了------------*/
            if (retry >= TX_RETRY_MAX) {
                // 超时丢帧（统计丢帧率，必要时报警或通知上层降频）
                // can_tx_drop_cnt++;
            }

        }
    }
}
