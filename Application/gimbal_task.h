#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

#include "pid.h"
#include "stdint.h"

#define MOTOR_ECD_TO_DEG 0.04395067757294591624954f //     360 /8192
// �궨ֵ���ֶ��ڵ�λ�ú��¼��д���ڴ�������flash��
#define YAW_ECD_LEFT    500     // ��Ӳ��λ ecd
#define YAW_ECD_RIGHT   4000    // ��Ӳ��λ ecd
#define YAW_ECD_CENTER  2250    // ���Ի�ͷ����ѡ��������ʾ�Ƕȣ�

// ��λ����
#define YAW_LIMIT_KP    0.03f    // ��������ϵ�����������ؿ�


typedef struct
{
    pid_t pid;
    float ref, fdb, err;
} gimbal_pid_t;

typedef struct
{
	float pitch_angle_max;
	float pitch_angle_min;
	float yaw_angle_max;
	float yaw_angle_min;
  float roll_angle_max;
	float roll_angle_min;

}gimbal_limit_t;

typedef struct
{
    gimbal_pid_t yaw_angle, yaw_spd;
    gimbal_pid_t pit_angle, pit_spd;
    gimbal_pid_t yaw_ecd  , yaw_spd_ecd;//云台编码
    gimbal_limit_t gimbal_limit;
    float yaw_output, pit_output;
    FeedForward_Typedef yaw_ff;
	float yaw_vel_feedforward;
} gimbal_t;

extern gimbal_t gimbal;
extern float yaw_err;
void gimbal_task(void const *argu);

#endif
