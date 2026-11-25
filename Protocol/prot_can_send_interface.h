/**
 * @file prot_can_send_interface.h
 * @brief 协议层 CAN 发送接口声明（抽象层）
 * @details 
 * 为什么新增这个文件？
 *   - 以前 prot 层直接调用 comm 或 HAL 的发送函数，导致层级耦合严重
 *   - 现在用这个接口文件做“中间桥梁”：只声明函数，不包含实现
 *   - prot 层只需要 #include 这个文件，就能调用发送功能
 *   - 好处：打破循环依赖；prot 层不再依赖 FreeRTOS/comm 内部细节
 *   - 以后换底层实现（队列→DMA→USB）只需要改 comm 层的实现，prot 层一行都不动
 *   - 这是一个典型的“接口隔离”设计（Interface Segregation）
 *   - 本车电机全部在 CAN1，只保留 1 个发送接口（2026-11-25 删除 CAN2/CAN3 接口）
 */

#ifndef __PROT_CAN_SEND_INTERFACE_H__
#define __PROT_CAN_SEND_INTERFACE_H__

#include <stdint.h>
#include "bsp_can.h"


typedef enum {
    PROT_CAN_CHANNEL_1 = (int)CAN_CHANNEL_1,  // 底盘 2006 ×4
    PROT_CAN_CHANNEL_2 = (int)CAN_CHANNEL_2   // 机械臂 6020（ID4，反馈0x208）
} prot_can_channel_e;   // 前缀加 prot_，表示这是协议层的抽象





/**
 * @brief 发送一帧到 CAN1（底盘 2006 或机械臂 6020）
 * @param id   CAN 帧 ID
 * @param data 数据指针（最多8字节）
 * @param dlc  数据长度（0~8）
 */
void Prot_CAN1_Send_Frame(uint32_t id, const uint8_t *data, uint8_t dlc);
void Prot_CAN2_Send_Frame(uint32_t id, const uint8_t *data, uint8_t dlc);

#endif /* __PROT_CAN_SEND_INTERFACE_H__ */
