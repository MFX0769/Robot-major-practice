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
 *   - 由于项目有 3 个 CAN 通道，ID 没有规律，无法统一判断，因此拆成 3 个独立接口函数
 *     每个函数对应一个通道，调用时明确知道发到哪个 CAN，避免歧义和耦合
 */

#ifndef __PROT_CAN_SEND_INTERFACE_H__
#define __PROT_CAN_SEND_INTERFACE_H__

#include <stdint.h>


typedef enum {
    PROT_CAN_CHANNEL_1 = 0,  // yaw、摩擦轮等
    PROT_CAN_CHANNEL_2 = 1,  // IMU
    PROT_CAN_CHANNEL_3 = 2   // pitch 等
} prot_can_channel_e;   // 前缀加 prot_，表示这是协议层的抽象





/**
 * @brief 发送一帧到 CAN1（例如 yaw、摩擦轮、拨弹盘电机）
 * @param id   CAN 帧 ID
 * @param data 数据指针（最多8字节）
 * @param dlc  数据长度（0~8）
 */
void Prot_CAN1_Send_Frame(uint32_t id, const uint8_t *data, uint8_t dlc);

/**
 * @brief 发送一帧到 CAN2（例如 IMU 数据）
 * @param id   CAN 帧 ID
 * @param data 数据指针（最多8字节）
 * @param dlc  数据长度（0~8）
 */
void Prot_CAN2_Send_Frame(uint32_t id, const uint8_t *data, uint8_t dlc);

/**
 * @brief 发送一帧到 CAN3（例如 pitch 电机等）
 * @param id   CAN 帧 ID
 * @param data 数据指针（最多8字节）
 * @param dlc  数据长度（0~8）
 */
void Prot_CAN3_Send_Frame(uint32_t id, const uint8_t *data, uint8_t dlc);

#endif /* __PROT_CAN_SEND_INTERFACE_H__ */
