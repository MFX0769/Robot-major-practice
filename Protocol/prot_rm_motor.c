#include "prot_rm_motor.h"
#include "string.h"


#ifndef PI
#define PI 3.14159265358979323846f
#endif
#define DATA_RANGE 0
#define CURRENT_RANGE 1
#define TORQUE_CONSTANT 2
#define REDUCTION_RATIO 3

// ȫ�ֱ�������
dji_motor_t fric_motor[2];
dji_motor_t pit_motor, yaw_motor;
dji_motor_t arm_yaw_motor;
dji_motor_t trigger_motor;
dji_motor_t chassis_2006[4];

can_std_msg_t motor_msg[CAN_CHANNEL_NUM][5];

static const float motor_para_table[3][4] =
{
    //�������ݷ�Χ            ������������������������Χ       ���س���                        ���Ĭ�ϼ��ٱ�
    {DJI_2006_MOTOR_DATA_RANGE, DJI_2006_MOTOR_CURRENT_RANGE, DJI_2006_MOTOR_TORQUE_CONSTANT, DJI_2006_ORIGINAL_REDUCTION_RATIO},
    {DJI_3508_MOTOR_DATA_RANGE, DJI_3508_MOTOR_CURRENT_RANGE, DJI_3508_MOTOR_TORQUE_CONSTANT, DJI_3508_ORIGINAL_REDUCTION_RATIO},
    {DJI_6020_MOTOR_DATA_RANGE, DJI_6020_MOTOR_CURRENT_RANGE, DJI_6020_MOTOR_TORQUE_CONSTANT, DJI_6020_ORIGINAL_REDUCTION_RATIO},
};

static void rm_motor_can_interface_send(dji_motor_t *motor, uint32_t id, const uint8_t *data);

/**
 * @brief     RM电机协议层对CAN发送接口的封装   Encapsulation of the CAN sending interface at the protocol layer of RM motor
 * @param[in] *motor   : RM电机结构体指针   Pointer to RM motor structure
 * @param[in] id      : CAN帧ID  CAN frame ID
 * @param[in] data    : CAN帧数据  CAN frame data
 * @param[in] dlc     : CAN帧长度  CAN frame length	
 * @retval    void
 * @note	  该函数调用prot_can_send_interface.h中声明的CAN发送接口函数,如果要修改CAN发送接口,只需修改电机结构体里面的can_channel即可自动实现切换
 *            This function calls the CAN sending interface function declared in prot_can_send_interface.h. If you want to modify the CAN sending interface, you only need to modify the can_channel in the motor structure to automatically switch.
 */
static void rm_motor_can_interface_send(dji_motor_t *motor, uint32_t id, const uint8_t *data)
{
	switch (motor->prot_can_channel)
	{
		case PROT_CAN_CHANNEL_1:
			Prot_CAN1_Send_Frame(id, data, 8);
			break;
		case PROT_CAN_CHANNEL_2:
			Prot_CAN2_Send_Frame(id, data, 8);
			break;
		case PROT_CAN_CHANNEL_3:
			Prot_CAN3_Send_Frame(id, data, 8);
			break;
		default:
			// 错误处理：无效的 CAN 通道
			break;
	}	
}



/*
 * @brief     �󽮵����ʼ������
 * @param[in] motor          : ������ݽṹ��
 * @param[in] motor_type     : �������
 * @param[in] can_channel    : �������canͨ��
 * @param[in] id             : ���id
 * @param[in] reduction_ratio: ���ʵ�ʼ��ٱ�
 * @retval    void
 */
void dji_motor_init(dji_motor_t *motor, uint8_t motor_type, prot_can_channel_e prot_can_channel, uint32_t id, float reduction_ratio)
{
    motor->motor_type = motor_type;
    motor->prot_can_channel = prot_can_channel;
    motor->can_id = id;
    motor->reduction_ratio = reduction_ratio;
}

/*
 * @brief     ���մ󽮵�����ݣ�����Ϊ���ʵ�λ
 * @param[in] motor: ������ݽṹ��
 * @param[in] data : ����ָ��
 * @retval    void
 */
void dji_motor_get_single_data(dji_motor_t * motor, uint8_t *data)
{
    motor->receive_cnt++;
    motor->online = 1;
    motor->err_percent  = ((float)motor->send_cnt - (float)motor->receive_cnt) / (float)motor->send_cnt;
    motor->last_ecd     = motor->ecd;
    motor->ecd          = (uint16_t)(data[0] << 8 | data[1]);
    motor->speed_rpm    = (int16_t)(data[2] << 8 | data[3]);
    motor->rx_current   = (int16_t)(data[4] << 8 | data[5]);
    motor->temperature  = data[6];
    if (motor->receive_cnt < 50) {
        motor->offset_ecd = motor->ecd;
        motor->round_cnt = 0;
    }
    if (motor->ecd - motor->last_ecd >= 4096)
        motor->round_cnt--;
    else if (motor->ecd - motor->last_ecd <= -4096)
        motor->round_cnt++;
    motor->total_ecd = motor->round_cnt * 8192 + motor->ecd - motor->offset_ecd;
    
    motor->p = 2.0f * PI * motor->total_ecd / 8192 / motor->reduction_ratio;
	motor->real_p = 2.0f * PI * motor->ecd / 8192 / motor->reduction_ratio;
    motor->v = 2.0f * PI * motor->speed_rpm / 60 / motor->reduction_ratio;
    motor->t = (float)motor->rx_current \
				* motor->reduction_ratio \
				/ motor_para_table[motor->motor_type][DATA_RANGE] \
				* motor_para_table[motor->motor_type][CURRENT_RANGE] \
				* motor_para_table[motor->motor_type][TORQUE_CONSTANT] \
				/ motor_para_table[motor->motor_type][REDUCTION_RATIO];
}

/**
  * @brief     get motor initialize offset value
  * @param     ptr: Pointer to a moto_measure_t structure
  * @retval    None
  * @attention this function should be called after system can init
  */
void get_moto_offset(dji_motor_t * ptr, FDCAN_HandleTypeDef* hcan, uint8_t * CAN_Rx_data)
{
    ptr->ecd        = (uint16_t)(CAN_Rx_data[0] << 8 | CAN_Rx_data[1]);
    ptr->offset_ecd = ptr->ecd;
}

/*
 * @brief     ���õ��ת��
 * @param[in] motor: ������ݽṹ��
 * @param[in] t    : Ŀ��ת�أ�N*m��
 * @retval    void
 */
void dji_motor_set_torque(dji_motor_t *motor, float t)
{
    motor->t = t;
    
    // ����Ŀ��ת�ؼ��㷢�͵���ֵ
    float current = t * motor_para_table[motor->motor_type][REDUCTION_RATIO] \
                    / motor_para_table[motor->motor_type][TORQUE_CONSTANT] \
                    / motor->reduction_ratio;
    
    motor->tx_current = (int16_t)(current * motor_para_table[motor->motor_type][DATA_RANGE] \
                    / motor_para_table[motor->motor_type][CURRENT_RANGE]);
    
    // ���Ʒ������ݷ�Χ
    int16_t max_range = (int16_t)motor_para_table[motor->motor_type][DATA_RANGE];
    if (motor->tx_current > max_range)
        motor->tx_current = max_range;
    else if (motor->tx_current < -max_range)
        motor->tx_current = -max_range;
}

/*������ר������󽮵���ķ��ͺ���*/
void can1_send_message(int16_t TX_ID, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
    uint8_t FreeTxNum = 0;
	FDCAN_TxHeaderTypeDef Tx1Message;
    uint8_t CAN1_Tx_data[8];
    Tx1Message.Identifier = TX_ID;
    Tx1Message.IdType = FDCAN_STANDARD_ID;
    Tx1Message.TxFrameType = FDCAN_DATA_FRAME;
    Tx1Message.DataLength = FDCAN_DLC_BYTES_8;

    CAN1_Tx_data[0] = iq1 >> 8;
    CAN1_Tx_data[1] = iq1;
    CAN1_Tx_data[2] = iq2 >> 8 ;
    CAN1_Tx_data[3] = iq2;
    CAN1_Tx_data[4] = iq3 >> 8;
    CAN1_Tx_data[5] = iq3;
    CAN1_Tx_data[6] = iq4 >> 8;
    CAN1_Tx_data[7] = iq4;

	
	uint8_t block_cnt = 0;
    //��ѯ���������Ƿ�Ϊ��
   FreeTxNum = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
    while(FreeTxNum == 0)
    {
        block_cnt++;
        FreeTxNum = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
        if(block_cnt >= 20)
        {
            return;  // ��ʱ�˳����������
        }
    }

    // ������Ϣ
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &Tx1Message, CAN1_Tx_data);
}


/*
 * @brief     ���󽮵�������������������ͻ�����
 * @param[in] motor: ������ݽṹ��
 * @retval    void
 */
static void dji_motor_fill_data(dji_motor_t *motor)
{
    if(motor == NULL)
        return;
    
    if(motor->can_id < 0x205)
    {
        // 0x201-0x204 �����
        motor_msg[motor->prot_can_channel][0].id = 0x200;
        motor_msg[motor->prot_can_channel][0].data[(motor->can_id - 0x201) * 2] = motor->tx_current >> 8;
        motor_msg[motor->prot_can_channel][0].data[(motor->can_id - 0x201) * 2 + 1] = motor->tx_current;
    }
    else if(motor->can_id < 0x209)
    {
        // 0x205-0x208 �����
        if(motor->prot_can_channel == PROT_CAN_CHANNEL_1)
            motor_msg[motor->prot_can_channel][1].id = 0x1ff;
        else 
            motor_msg[motor->prot_can_channel][1].id = 0x1ff;
        
        motor_msg[motor->prot_can_channel][1].data[(motor->can_id - 0x205) * 2] = motor->tx_current >> 8;
        motor_msg[motor->prot_can_channel][1].data[(motor->can_id - 0x205) * 2 + 1] = motor->tx_current;
		
	}
    
    motor->send_cnt++;
}

/*
 * @brief     1��4�󽮵�����͵���
 * @param[in] motor1~4: �ֱ��Ӧ�ĸ����
 * @retval    void
 */
void dji_send_current(dji_motor_t *motor1, dji_motor_t *motor2, dji_motor_t *motor3, dji_motor_t *motor4)
{
    memset(&motor_msg, 0, sizeof(motor_msg));
    
    dji_motor_fill_data(motor1);
    dji_motor_fill_data(motor2);
    dji_motor_fill_data(motor3);
    dji_motor_fill_data(motor4);	
    
    // ��������
    if(motor1 != NULL && motor1->can_id < 0x205)
    {
        // bsp_can_std_transmit(       motor1->can_channel,      motor_msg[motor1->can_channel][0].id, motor_msg[motor1->can_channel][0].data);
        rm_motor_can_interface_send(motor1, motor_msg[motor1->prot_can_channel][0].id, motor_msg[motor1->prot_can_channel][0].data);
    }
    else if(motor1 != NULL && motor1->can_id < 0x209)
    {
        // bsp_can_std_transmit(       motor1->can_channel,      motor_msg[motor1->can_channel][4].id, motor_msg[motor1->can_channel][4].data);
        rm_motor_can_interface_send(motor1, motor_msg[motor1->prot_can_channel][1].id, motor_msg[motor1->prot_can_channel][1].data);

    }
}
