/**
 * @file prot_rm_motor.h
 * @brief 机械臂/底盘 DJI 电机接口：2006 ×4，反馈 ID 0x201~0x204；6020 ×1，反馈 ID 0x208。
 */
#ifndef __PROT_RM_MOTOR_H
#define __PROT_RM_MOTOR_H

#include "bsp_can.h"
#include "can_comm.h"
#include "prot_can_send_interface.h"

#define DJI_2006_MOTOR 0x00U
#define DJI_3508_MOTOR 0x01U
#define DJI_6020_MOTOR 0x02U

#define DJI_2006_MOTOR_DATA_RANGE 10000
#define DJI_3508_MOTOR_DATA_RANGE 16384
#define DJI_6020_MOTOR_DATA_RANGE 25000
#define DJI_2006_MOTOR_CURRENT_RANGE 10.0f
#define DJI_3508_MOTOR_CURRENT_RANGE 20.0f
#define DJI_6020_MOTOR_CURRENT_RANGE 1.0f
#define DJI_2006_MOTOR_TORQUE_CONSTANT 0.18f
#define DJI_3508_MOTOR_TORQUE_CONSTANT 0.246f
#define DJI_6020_MOTOR_TORQUE_CONSTANT 1.0f
#define DJI_2006_ORIGINAL_REDUCTION_RATIO 36
#define DJI_3508_ORIGINAL_REDUCTION_RATIO 19.2f
#define DJI_6020_ORIGINAL_REDUCTION_RATIO 1

#define CHA_LEFT_FRONT_2006  0x204U
#define CHA_LEFT_BACK_2006   0x202U
#define CHA_RIGHT_FRONT_2006 0x203U
#define CHA_RIGHT_BACK_2006  0x201U

typedef struct
{
    uint8_t motor_type;
    prot_can_channel_e prot_can_channel;
    uint32_t can_id;
    float reduction_ratio;
    uint32_t send_cnt, receive_cnt;
    float err_percent;
    uint8_t online;
    int16_t tx_current;
    float p_ref, v_ref;
    int16_t ecd_ref;
    float p, v, t;
    float real_p;
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t rx_current;
    uint8_t temperature;
    uint16_t offset_ecd, last_ecd;
    int32_t total_ecd;
    int32_t round_cnt;
} dji_motor_t;

typedef struct
{
    uint32_t id;
    uint8_t data[8];
} can_std_msg_t;

extern dji_motor_t chassis_2006[4];
extern dji_motor_t arm_yaw_motor;

void dji_motor_init(dji_motor_t *motor, uint8_t motor_type,
                    prot_can_channel_e prot_can_channel, uint32_t id,
                    float reduction_ratio);
void dji_motor_get_single_data(dji_motor_t *motor, uint8_t *data);
void dji_motor_set_torque(dji_motor_t *motor, float t);
void dji_send_current(dji_motor_t *motor1, dji_motor_t *motor2,
                      dji_motor_t *motor3, dji_motor_t *motor4);

#endif
