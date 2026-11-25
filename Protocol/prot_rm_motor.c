/**
 * @file prot_rm_motor.c
 * @brief DJI 2006/6020 协议层。5 个电机全部在 CAN1，发送使用局部帧避免共享缓冲区越界。
 */
#include "prot_rm_motor.h"
#include "string.h"

#ifndef PI
#define PI 3.14159265358979323846f
#endif

#define DATA_RANGE 0
#define CURRENT_RANGE 1
#define TORQUE_CONSTANT 2
#define REDUCTION_RATIO 3

dji_motor_t chassis_2006[4];
dji_motor_t arm_yaw_motor;

static const float motor_para_table[3][4] =
{
    {DJI_2006_MOTOR_DATA_RANGE, DJI_2006_MOTOR_CURRENT_RANGE, DJI_2006_MOTOR_TORQUE_CONSTANT, DJI_2006_ORIGINAL_REDUCTION_RATIO},
    {DJI_3508_MOTOR_DATA_RANGE, DJI_3508_MOTOR_CURRENT_RANGE, DJI_3508_MOTOR_TORQUE_CONSTANT, DJI_3508_ORIGINAL_REDUCTION_RATIO},
    {DJI_6020_MOTOR_DATA_RANGE, DJI_6020_MOTOR_CURRENT_RANGE, DJI_6020_MOTOR_TORQUE_CONSTANT, DJI_6020_ORIGINAL_REDUCTION_RATIO},
};

static void can_send(const dji_motor_t *motor, uint32_t id, const uint8_t data[8])
{
    if (motor->prot_can_channel == PROT_CAN_CHANNEL_2)
        (void)Prot_CAN2_Send_Frame(id, data, 8U);
    else
        (void)Prot_CAN1_Send_Frame(id, data, 8U);
}

void dji_motor_init(dji_motor_t *motor, uint8_t motor_type,
                    prot_can_channel_e prot_can_channel, uint32_t id,
                    float reduction_ratio)
{
    if (motor == NULL || motor_type > DJI_6020_MOTOR ||
        (prot_can_channel != PROT_CAN_CHANNEL_1 && prot_can_channel != PROT_CAN_CHANNEL_2) ||
        id < 0x201U || id > 0x208U || reduction_ratio <= 0.0f)
    {
        return;
    }
    memset(motor, 0, sizeof(*motor));
    motor->motor_type = motor_type;
    motor->prot_can_channel = prot_can_channel;
    motor->can_id = id;
    motor->reduction_ratio = reduction_ratio;
}

void dji_motor_get_single_data(dji_motor_t *motor, uint8_t *data)
{
    if (motor == NULL || data == NULL) return;

    motor->receive_cnt++;
    motor->online = 1;
    if (motor->send_cnt != 0U)
    {
        motor->err_percent = ((float)motor->send_cnt - (float)motor->receive_cnt) /
                             (float)motor->send_cnt;
    }
    motor->last_ecd = motor->ecd;
    motor->ecd = (uint16_t)((data[0] << 8) | data[1]);
    motor->speed_rpm = (int16_t)((data[2] << 8) | data[3]);
    motor->rx_current = (int16_t)((data[4] << 8) | data[5]);
    motor->temperature = data[6];
    if (motor->receive_cnt < 50U)
    {
        motor->offset_ecd = motor->ecd;
        motor->round_cnt = 0;
    }
    if (motor->ecd - motor->last_ecd >= 4096) motor->round_cnt--;
    else if (motor->ecd - motor->last_ecd <= -4096) motor->round_cnt++;
    motor->total_ecd = motor->round_cnt * 8192 + motor->ecd - motor->offset_ecd;
    motor->p = 2.0f * PI * motor->total_ecd / 8192 / motor->reduction_ratio;
    motor->real_p = 2.0f * PI * motor->ecd / 8192 / motor->reduction_ratio;
    motor->v = 2.0f * PI * motor->speed_rpm / 60 / motor->reduction_ratio;
    motor->t = (float)motor->rx_current * motor->reduction_ratio /
               motor_para_table[motor->motor_type][DATA_RANGE] *
               motor_para_table[motor->motor_type][CURRENT_RANGE] *
               motor_para_table[motor->motor_type][TORQUE_CONSTANT] /
               motor_para_table[motor->motor_type][REDUCTION_RATIO];
}

void dji_motor_set_torque(dji_motor_t *motor, float t)
{
    if (motor == NULL || motor->motor_type > DJI_6020_MOTOR) return;

    motor->t = t;
    float current = t * motor_para_table[motor->motor_type][REDUCTION_RATIO] /
                    motor_para_table[motor->motor_type][TORQUE_CONSTANT] /
                    motor->reduction_ratio;
    current *= motor_para_table[motor->motor_type][DATA_RANGE] /
               motor_para_table[motor->motor_type][CURRENT_RANGE];
    int16_t range = (int16_t)motor_para_table[motor->motor_type][DATA_RANGE];
    if (current > range) current = range;
    else if (current < -range) current = -range;
    motor->tx_current = (int16_t)current;
}

static void fill_current(can_std_msg_t *frame, dji_motor_t *motor)
{
    uint8_t slot;
    if (motor->can_id <= 0x204U)
    {
        frame->id = 0x200U;
        slot = (uint8_t)(motor->can_id - 0x201U);
    }
    else
    {
        frame->id = 0x1FFU;
        slot = (uint8_t)(motor->can_id - 0x205U);
    }
    frame->data[slot * 2U] = (uint8_t)(motor->tx_current >> 8);
    frame->data[slot * 2U + 1U] = (uint8_t)motor->tx_current;
    motor->send_cnt++;
}

void dji_send_current(dji_motor_t *motor1, dji_motor_t *motor2,
                      dji_motor_t *motor3, dji_motor_t *motor4)
{
    can_std_msg_t low = {.id = 0x200U, .data = {0}};
    can_std_msg_t high = {.id = 0x1FFU, .data = {0}};
    dji_motor_t *motors[4] = {motor1, motor2, motor3, motor4};
    dji_motor_t *low_owner = NULL;
    dji_motor_t *high_owner = NULL;

    for (uint8_t i = 0; i < 4U; i++)
    {
        if (motors[i] == NULL) continue;
        if (motors[i]->can_id <= 0x204U)
        {
            fill_current(&low, motors[i]);
            low_owner = motors[i];
        }
        else if (motors[i]->can_id <= 0x208U)
        {
            fill_current(&high, motors[i]);
            high_owner = motors[i];
        }
    }
    if (low_owner != NULL) can_send(low_owner, low.id, low.data);
    if (high_owner != NULL) can_send(high_owner, high.id, high.data);
}
