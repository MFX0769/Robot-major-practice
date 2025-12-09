#ifndef __PROT_DM_MOTOR_H
#define __PROT_DM_MOTOR_H
#include "stm32h7xx.h"                  // Device header
#include "bsp_can.h"
#include "can_comm.h"

//限幅
#ifndef LIMIT_MIN_MAX
	#define LIMIT_MIN_MAX(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x)))
#endif
//控制命令
#define DM_ENABLE_CMD  0x01
#define DM_DISABLE_CMD 0x02
#define DM_ZERO_CMD    0x03
#define DM_CLEAR_CMD   0x04
//电机发送ID
#define DM_PITCH_SEND_ID 0x06
#define DM_3507_SEND_ID 0x07
//电机接收ID
#define DM_PITCH_RECEIVE_ID 0X016
#define DM_3507_RECEIVE_ID 0x07
//电机减速比
#define DM_4310_RATIO  10.0f
#define DM_3507_RATIO 7.0f
//电机安装零点偏置
#define DM_PITCH_OFFSET  0
#define DM_3507_OFFEST 0
//达妙电机的型号
#define DM4310   0x01
#define DM3507 0x02
//DM电机参数范围
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
#define DM_KP_MIN        0.0f		//Kp最小值
#define DM_KP_MAX      500.0f		//Kp最大值
#define DM_KD_MIN        0.0f		//Kd最小值
#define DM_KD_MAX        5.0f		//Kd最大值
//达妙电机结构体
typedef struct
{
	//电机参数
	uint8_t motor_type;
	can_channel_e can_channel;	
	uint32_t can_receive_id,can_send_id;
	uint32_t send_cnt, receive_cnt;
	float err_percent;
	//安装角度补偿
	float zero_point;	
	//电机减速比
	float ratio;
	//控制数据mit
	float p_ref, v_ref, t_ref, kp, kd;
	//纯力矩
	float p_set, v_set, t_set;
	//反馈数据
	float last_p, p, v, t;
	uint8_t err_code;
	uint8_t online;
	//有传动比的情况下，反馈数据换算到输出轴
	float p_tmp,real_p,real_v;
}dm_motor_t;
extern dm_motor_t pitch_motor;
extern dm_motor_t dm3507_motor;

void dm_motor_init(dm_motor_t *motor,uint8_t motor_type,can_channel_e can_channel, uint32_t r_id, uint32_t t_id, float ratio,float zero_point);
void dm_set_cmd(dm_motor_t *motor,uint8_t cmd);
void dm_mit_set_para(dm_motor_t *motor,float p,float v,float t,float kp,float kd);
void dm_mit_control(dm_motor_t *motor);
void dm_get_single_data(dm_motor_t *motor,uint8_t *data);
void dm_mit_set_torque(dm_motor_t *motor,float ff_out,float pos_out);

extern dm_motor_t pitch_motor;

#endif
