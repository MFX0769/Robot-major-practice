#include "prot_dm_motor.h"

dm_motor_t pitch_motor;
dm_motor_t dm3507_motor;


/**
 * @brief     ´ïÃîµç»úÐ­Òé²ã¶ÔCAN·¢ËÍ½Ó¿ÚµÄ·â×°   Encapsulation of the CAN sending interface at the protocol layer of DM motor
 * @param[in] *motor   : ´ïÃîµç»ú½á¹¹ÌåÖ¸Õë   Pointer to DM motor structure
 * @param[in] id      : CANÖ¡ID  CAN frame ID
 * @param[in] data    : CANÖ¡Êý¾Ý  CAN frame data
 * @param[in] dlc     : CANÖ¡³¤¶È  CAN frame length	
 * @retval    void
 * @note	  ¸Ãº¯Êýµ÷ÓÃprot_can_send_interface.hÖÐÉùÃ÷µÄCAN·¢ËÍ½Ó¿Úº¯Êý,Èç¹ûÒªÐÞ¸ÄCAN·¢ËÍ½Ó¿Ú,Ö»ÐèÐÞ¸Äµç»ú½á¹¹ÌåÀïÃæµÄcan_channel¼´¿É×Ô¶¯ÊµÏÖÇÐ»»
 *            This function calls the CAN sending interface function declared in prot_can_send_interface.h. If you want to modify the CAN sending interface, you only need to modify the can_channel in the motor structure to automatically switch.
 */
static void dm_motor_can_interface_send(dm_motor_t *motor, uint32_t id, const uint8_t *data)
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
			// ´íÎó´¦Àí£ºÎÞÐ§µÄ CAN Í¨µÀ
			break;
	}	
}



/**
 * @brief    	ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
 * @param[in] *motor    : ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ý½á¹¹ï¿½ï¿½Ö¸ï¿½ï¿?
 * @param[in] motor_type: ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿?
 * @param[in] can_periph: ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½canÍ¨ï¿½ï¿½
 * @param[in] r_id      : Master_id
 * @param[in] t_id      : Can_id
 * @param[in] ratio     : ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ù±ï¿?
 * @param[in] zero_point: ï¿½ï¿½ï¿½ï¿½ï¿½×°ï¿½ï¿½ï¿½
 * @retval    void
 */
void dm_motor_init(dm_motor_t *motor,uint8_t motor_type,can_channel_e can_channel, uint32_t r_id, uint32_t t_id, float ratio,float zero_point)
{
	motor->prot_can_channel = can_channel;
    motor->can_receive_id = r_id;
	motor->can_send_id = t_id;
    motor->ratio = ratio;
	motor->motor_type = motor_type;
	motor->zero_point = zero_point;
	
	dm_set_cmd(motor,DM_ENABLE_CMD);
	HAL_Delay(1);
	dm_mit_set_para(motor,0,0,0,0,0);
	dm_mit_control(motor);
	HAL_Delay(1);
}
/*
* @brief    	ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
 * @param[in] *motor    : ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ý½á¹¹ï¿½ï¿½Ö¸ï¿½ï¿?
 * @param[in] cmd		: ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿?
 * @retval    void
 */
void dm_set_cmd(dm_motor_t *motor,uint8_t cmd)
{
	uint8_t buf[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00};
	switch(cmd)
	{
		case DM_ENABLE_CMD://Ê¹ï¿½ï¿½
		{
			buf[7] = 0xFC;
		}break;
		case DM_DISABLE_CMD://Ê§ï¿½ï¿½
		{
			buf[7] = 0xFD;
		}break;
		case DM_ZERO_CMD://ï¿½ï¿½ï¿½ï¿½
		{
			buf[7] = 0xFE;
		}break;
		case DM_CLEAR_CMD://ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿?
		{
			buf[7] = 0xFB;
		}break;
		default:return;
	}
	// bsp_can_std_transmit(motor->can_channel,motor->can_send_id,buf);
	dm_motor_can_interface_send(motor, motor->can_send_id, buf);
}
/*
* @brief    	ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½MITï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
 * @param[in] *motor    : ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ý½á¹¹ï¿½ï¿½Ö¸ï¿½ï¿?
 * @param[in] p     	: ï¿½ï¿½ï¿½Ä¿ï¿½ï¿½Î»ï¿½ï¿?
 * @param[in] v     	: ï¿½ï¿½ï¿½Ä¿ï¿½ï¿½ï¿½Ù¶ï¿?
 * @param[in] t 	    : ï¿½ï¿½ï¿½Ä¿ï¿½ï¿½ï¿½ï¿½ï¿½ï¿?
 * @param[in] kp     	: ï¿½ï¿½ï¿½kp
 * @param[in] kd     	: ï¿½ï¿½ï¿½kd
 * @retval    void
 */
void dm_mit_set_para(dm_motor_t *motor,float p,float v,float t,float kp,float kd)
{
	motor->p_ref = p;	//ï¿½è¶¨Ä¿ï¿½ï¿½Î»ï¿½ï¿½
	motor->v_ref = v;	//ï¿½è¶¨Ä¿ï¿½ï¿½ï¿½Ù¶ï¿½
	motor->t_ref = t;	//ï¿½è¶¨Ä¿ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
	motor->kp = kp;		
	motor->kd = kd;
	//refï¿½Þ·ï¿½
	LIMIT_MIN_MAX(motor->p_ref, DM_P_MIN, DM_P_MAX);
    LIMIT_MIN_MAX(motor->v_ref, DM_V_MIN, DM_V_MAX);
    LIMIT_MIN_MAX(motor->kp, DM_KP_MIN, DM_KP_MAX);
    LIMIT_MIN_MAX(motor->kd, DM_KD_MIN, DM_KD_MAX);
	if(motor->motor_type == DM4310)
		LIMIT_MIN_MAX(motor->t_ref, DM4310_T_MIN, DM4310_T_MAX);
	else if(motor->motor_type == DM3507)
		LIMIT_MIN_MAX(motor->t_ref, DM3507_T_MIN, DM3507_T_MAX);

}
/*
* @brief    	ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½MITÄ£Ê½ï¿½Â´ï¿½ï¿½ï¿½ï¿½Ø¿ï¿½ï¿½ï¿½
 * @param[in] *motor    : ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ý½á¹¹ï¿½ï¿½Ö¸ï¿½ï¿?
 * @param[in] out     	: ï¿½ï¿½ï¿½pidï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö?
 * @param[in] ff 	    : ï¿½ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿?
 * @retval    void
 */
void dm_mit_set_torque(dm_motor_t *motor,float ff_out,float pos_out)
{
	dm_mit_set_para(motor,0,0,ff_out + pos_out,0,0);
}
/*
* @brief    	ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½MITï¿½ï¿½ï¿½ï¿½
 * @param[in] *motor     : ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ý½á¹¹ï¿½ï¿½Ö¸ï¿½ï¿?
 * @retval    void
 */
void dm_mit_control(dm_motor_t *motor)
{
	if(motor->err_code == 0)//ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê§ï¿½ï¿½
	{
		dm_set_cmd(motor,DM_ENABLE_CMD);
		return;
	}
	else if(motor->err_code != 0 && motor->err_code != 1 && motor->err_code != 5)//ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê§ï¿½Ü£ï¿½Ê¹ï¿½ï¿½×´Ì?ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿?
	{
		dm_set_cmd(motor,DM_CLEAR_CMD);
		return;
	}
	static uint8_t buf[8];
	uint16_t p_tmp,v_tmp,kp_tmp,kd_tmp,t_tmp;
	p_tmp = float_to_uint(motor->p_ref, DM_P_MIN, DM_P_MAX, 16);
	v_tmp = float_to_uint(motor->v_ref, DM_V_MIN, DM_V_MAX, 12);
	switch(motor->motor_type)
	{
		case DM4310:
		{
			t_tmp = float_to_uint(motor->t_ref, DM4310_T_MIN,DM4310_T_MAX, 12);
		}break;
		case DM3507:
		{
			t_tmp = float_to_uint(motor->t_ref, DM3507_T_MIN,DM3507_T_MAX, 12);
		}break;
		
		default:break;
	}
	kp_tmp = float_to_uint(motor->kp, DM_KP_MIN, DM_KP_MAX, 12);
	kd_tmp = float_to_uint(motor->kd, DM_KD_MIN, DM_KD_MAX, 12);
	
	buf[0] = (p_tmp >> 8);
	buf[1] = p_tmp;
	buf[2] = (v_tmp >> 4);
	buf[3] = ((v_tmp&0xF)<<4)|(kp_tmp>>8);
	buf[4] = kp_tmp;
	buf[5] = (kd_tmp >> 4);
	buf[6] = ((kd_tmp&0xF)<<4)|(t_tmp>>8);
	buf[7] = t_tmp;
	
	// if(bsp_can_std_transmit(motor->can_channel,motor->can_send_id,buf) == HAL_OK)
//	{
//		motor->send_cnt++;
//		motor->online = 0;
//	}
	
	dm_motor_can_interface_send(motor, motor->can_send_id, buf);
	motor->send_cnt++;
	motor->online = 0;
}
/*
 * @brief    	ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ý½ï¿½ï¿½Õ´ï¿½ï¿½ï¿½
 * @param[in] *motor    : ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ý½á¹¹ï¿½ï¿½Ö¸ï¿½ï¿?
 * @param[in] *data		: ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö¸ï¿½ï¿?
 * @retval    void
 */
void dm_get_single_data(dm_motor_t *motor,uint8_t *data)
{
	uint16_t p_int,v_int,t_int;
    motor->receive_cnt++;
	motor->online = 1;
    motor->err_percent = ((float)motor->send_cnt - (float)motor->receive_cnt) / (float)motor->send_cnt;
	//ï¿½ï¿½È¡ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½×´Ì?ï¿½ï¿½Ï¢
	motor->err_code = data[0]>> 4;
	//ï¿½ï¿½ï¿½ï¿½Ð­ï¿½é£¬ï¿½ï¿½uintï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½×ªï¿½ï¿½
    p_int = (data[1] << 8) | data[2];
	motor->p = uint_to_float(p_int,DM_P_MIN,DM_P_MAX,16);
			  
	v_int = (data[3]<<4|data[4]>>4);
	motor->v = uint_to_float(v_int,DM_V_MIN,DM_V_MAX,12);

	t_int = ((data[4]&0xF)<<8)|data[5];
	switch(motor->motor_type)
	{
		case DM4310:
		{
			motor->t = uint_to_float(t_int,DM4310_T_MIN,DM4310_T_MAX,12);
		}break;
		case DM3507:
		{
			motor->t = uint_to_float(t_int,DM3507_T_MIN,DM3507_T_MAX,12);
		}break; 
		
		default:break;
	}		
	
	motor->last_p =  motor->p;
}

void dm_clear(dm_motor_t *motor)
{
	motor->p_ref = 0;
	motor->v_ref = 0;
	motor->t_ref = 0;
}
