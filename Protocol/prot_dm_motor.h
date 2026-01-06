#ifndef __PROT_DM_MOTOR_H
#define __PROT_DM_MOTOR_H
// #include "stm32h7xx.h"                  // Device header
// #include "bsp_can.h"


#include "can_comm.h"
#include "can_message.h"
#include "prot_can_send_interface.h"
#include "math_calcu.h"
//޷
#ifndef LIMIT_MIN_MAX
	#define LIMIT_MIN_MAX(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x)))
#endif
//��������
#define DM_ENABLE_CMD  0x01
#define DM_DISABLE_CMD 0x02
#define DM_ZERO_CMD    0x03
#define DM_CLEAR_CMD   0x04
//�������ID
#define DM_PITCH_SEND_ID 0x06
#define DM_3507_SEND_ID 0x07
//�������ID
#define DM_PITCH_RECEIVE_ID 0X016
#define DM_3507_RECEIVE_ID 0x07
//������ٱ�
#define DM_4310_RATIO  10.0f
#define DM_3507_RATIO 7.0f
//�����װ���ƫ��
#define DM_PITCH_OFFSET  0
#define DM_3507_OFFEST 0
//���������ͺ�
#define DM4310   0x01
#define DM3507 0x02
//DM���������Χ
#define DM_P_MAX        12.56637f
#define DM_P_MIN       -12.56637f
#define DM_V_MAX        45.0f
#define DM_V_MIN       -45.0f
#define DM3507_V_MAX        50.0f
#define DM3507_V_MIN       -50.0f
#define DM4310_T_MAX    18.0f
#define DM4310_T_MIN   -18.0f
#define DM3507_T_MAX	5.0f
#define DM3507_T_MIN   -5.0f
#define DM_KP_MIN        0.0f		//Kp��Сֵ
#define DM_KP_MAX      500.0f		//Kp���ֵ
#define DM_KD_MIN        0.0f		//Kd��Сֵ
#define DM_KD_MAX        5.0f		//Kd���ֵ
//�������ṹ��
typedef struct
{
	//�������
	uint8_t motor_type;
//	can_channel_e can_channel;	
	prot_can_channel_e prot_can_channel;
	uint32_t can_receive_id,can_send_id;
	uint32_t send_cnt, receive_cnt;
	float err_percent;
	//��װ�ǶȲ���
	float zero_point;	
	//������ٱ�
	float ratio;
	//��������mit
	float p_ref, v_ref, t_ref, kp, kd;
	//������
	float p_set, v_set, t_set;
	//��������
	float last_p, p, v, t;
	uint8_t err_code;
	uint8_t online;
	//�д����ȵ�����£��������ݻ��㵽�����
	float p_tmp,real_p,real_v;
}dm_motor_t;
extern dm_motor_t pitch_motor;
extern dm_motor_t dm3507_motor;

void dm_motor_init(dm_motor_t *motor,uint8_t motor_type,prot_can_channel_e can_channel, uint32_t r_id, uint32_t t_id, float ratio,float zero_point);
void dm_set_cmd(dm_motor_t *motor,uint8_t cmd);
void dm_mit_set_para(dm_motor_t *motor,float p,float v,float t,float kp,float kd);
void dm_mit_control(dm_motor_t *motor);
void dm_get_single_data(dm_motor_t *motor,uint8_t *data);
void dm_mit_set_torque(dm_motor_t *motor,float ff_out,float pos_out);
void dm_clear(dm_motor_t *motor);

extern dm_motor_t pitch_motor;

#endif
