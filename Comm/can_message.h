/**
 * @file can_message.h
 * @brief CAN 消息公共结构体定义
 * @details 
 * 为什么新增这个文件？
 *   - 原来的 COB_TypeDef 只在 comm 或 prot 里定义，容易造成重复定义或循环依赖
 *   - 现在把 CAN 帧的结构体独立出来，成为公共头文件
 *   - 好处：prot层和comm层都可以安全包含它，不引入额外依赖
 *   - 未来扩展（如加扩展帧、时间戳）也只需要改这里
 *   - 命名改成 CanFrame_t，更直观易懂（原 COB 是社区老习惯）
 * 
 * 使用范围：所有需要组包/发帧/收帧的模块
 */

#ifndef __CAN_MESSAGE_H__
#define __CAN_MESSAGE_H__

#include <stdint.h>






typedef struct {
    uint32_t ID;       // CAN ID（支持标准帧和扩展帧）
    uint8_t  DLC;      // 数据长度代码（0~8）
    uint8_t  Data[8];  // 数据字节数组（最多8字节）
} CanFrame_t;          // 推荐新名字：CanFrame_t，更直观

#endif /* __CAN_MESSAGE_H__ */

