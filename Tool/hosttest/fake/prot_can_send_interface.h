#ifndef FAKE_PROT_CAN_SEND_INTERFACE_H
#define FAKE_PROT_CAN_SEND_INTERFACE_H
#include <stdint.h>
typedef enum { PROT_CAN_CHANNEL_1 = 0, PROT_CAN_CHANNEL_2 = 1 } prot_can_channel_e;
/* 真固件里它们把帧放进 CAN 发送队列；测试台里改成直接记下这一帧（sim_extra.c） */
void Prot_CAN1_Send_Frame(uint32_t id, const uint8_t *data, uint8_t dlc);
void Prot_CAN2_Send_Frame(uint32_t id, const uint8_t *data, uint8_t dlc);
#endif
