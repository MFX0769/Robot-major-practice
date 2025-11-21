#include "prot_dm_motor.h"
#include "math_calcu.h"
dm_motor_t pitch_motor;

/*
 * @brief    	达妙电机初始化设置
 * @param[in] *motor    : 电机数据结构体指针
 * @param[in] motor_type: 电机类型
 * @param[in] can_periph: 电机所在can通道
 * @param[in] r_id      : Master_id
 * @param[in] t_id      : Can_id
 * @param[in] ratio     : 电机减速比
 * @param[in] zero_point: 电机安装零点
 * @retval    void
 */
void dm_motor_init(dm_motor_t *motor,uint8_t motor_type,can_channel_e can_channel, uint32_t r_id, uint32_t t_id, float ratio,float zero_point)
{
	motor->can_channel = can_channel;
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
* @brief    	达妙电机发送命令
 * @param[in] *motor    : 电机数据结构体指针
 * @param[in] cmd		: 电机命令
 * @retval    void
 */
void dm_set_cmd(dm_motor_t *motor,uint8_t cmd)
{
	uint8_t buf[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00};
	switch(cmd)
	{
		case DM_ENABLE_CMD://使能
		{
			buf[7] = 0xFC;
		}break;
		case DM_DISABLE_CMD://失能
		{
			buf[7] = 0xFD;
		}break;
		case DM_ZERO_CMD://置零
		{
			buf[7] = 0xFE;
		}break;
		case DM_CLEAR_CMD://清除错误
		{
			buf[7] = 0xFB;
		}break;
		default:return;
	}
	can_std_transmit(motor->can_channel,motor->can_send_id,buf);
}
/*
* @brief    	达妙电机MIT参数设置
 * @param[in] *motor    : 电机数据结构体指针
 * @param[in] p     	: 电机目标位置
 * @param[in] v     	: 电机目标速度
 * @param[in] t 	    : 电机目标力矩
 * @param[in] kp     	: 电机kp
 * @param[in] kd     	: 电机kd
 * @retval    void
 */
void dm_mit_set_para(dm_motor_t *motor,float p,float v,float t,float kp,float kd)
{
	motor->p_ref = p;	//设定目标位置
	motor->v_ref = v;	//设定目标速度
	motor->t_ref = t;	//设定目标力矩
	motor->kp = kp;		
	motor->kd = kd;
	//ref限幅
	LIMIT_MIN_MAX(motor->p_ref, DM_P_MIN, DM_P_MAX);
    LIMIT_MIN_MAX(motor->v_ref, DM_V_MIN, DM_V_MAX);
    LIMIT_MIN_MAX(motor->kp, DM_KP_MIN, DM_KP_MAX);
    LIMIT_MIN_MAX(motor->kd, DM_KD_MIN, DM_KD_MAX);
	if(motor->motor_type == DM4310)
		LIMIT_MIN_MAX(motor->t_ref, DM4310_T_MIN, DM4310_T_MAX);
//	else if(motor->motor_type == DM4340P)
//		LIMIT_MIN_MAX(motor->t_ref, DM4340P_T_MIN, DM4340P_T_MAX);

}
/*
* @brief    	达妙电机MIT模式下纯力矩控制
 * @param[in] *motor    : 电机数据结构体指针
 * @param[in] out     	: 电机pid计算出来的值
 * @param[in] ff 	    : 电机前馈力矩
 * @retval    void
 */
void dm_mit_set_torque(dm_motor_t *motor,float ff_out,float pos_out)
{
	dm_mit_set_para(motor,0,0,ff_out + pos_out,0,0);
}
/*
* @brief    	达妙电机MIT控制
 * @param[in] *motor     : 电机数据结构体指针
 * @retval    void
 */
void dm_mit_control(dm_motor_t *motor)
{
	if(motor->err_code == 0)//如果电机失能
	{
		dm_set_cmd(motor,DM_ENABLE_CMD);
		return;
	}
	else if(motor->err_code != 0 && motor->err_code != 1 && motor->err_code != 5)//如果不是失能，使能状态，就清除错误
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
//		case DM4340P:
//		{
//			t_tmp = float_to_uint(motor->t_ref, DM4340P_T_MIN,DM4340P_T_MAX, 12);
//		}break;
//		
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
	
	if(can_std_transmit(motor->can_channel,motor->can_send_id,buf) == HAL_OK)
	{
		motor->send_cnt++;
		motor->online = 0;
	}
}
/*
 * @brief    	达妙电机数据接收处理
 * @param[in] *motor    : 电机数据结构体指针
 * @param[in] *data		: 电机接收数据指针
 * @retval    void
 */
void dm_get_single_data(dm_motor_t *motor,uint8_t *data)
{
	uint16_t p_int,v_int,t_int;
    motor->receive_cnt++;
	motor->online = 1;
    motor->err_percent = ((float)motor->send_cnt - (float)motor->receive_cnt) / (float)motor->send_cnt;
	//获取电机错误状态信息
	motor->err_code = data[0]>> 4;
	//根据协议，对uint参数进行转换
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
//		case DM4340P:
//		{
//			motor->t = uint_to_float(t_int,DM4340P_T_MIN,DM4340P_T_MAX,12);
//		}break;
		
		default:break;
	}		
	
	motor->last_p =  motor->p;
}
