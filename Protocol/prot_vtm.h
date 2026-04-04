#ifndef __PROT_VTM_H__
#define __PROT_VTM_H__
/**
 * @file prot_vtm.h
 * @brief 图传模块(VTM)遥控数据协议 (RoboMaster VT03/VT13)
 * @details
 *   图传模块数据帧解析，包含：
 *   - 4通道摇杆数据 (ch0-ch3)
 *   - 拨轮数据 (dial)
 *   - 挡位切换开关 (cns)
 *   - 物理按键 (暂停、扳机、自定义左右)
 *   - 键盘鼠标数据
 *   - CRC16校验
 * 
 *   通信参数：
 *   - 波特率: 921600
 *   - 数据位: 8
 *   - 停止位: 1
 *   - 校验位: 无
 *   - 数据帧长度: 21字节
 *   - 数据帧间隔: 14ms
 * 
 * @date 2026
 */

#include "stdint.h"
#include "prot_dr16.h"
#define VTM_DATA_LEN    21

#pragma pack(1)

/* VTM遥控数据结构 (原始值) */
typedef struct
{
    /* 摇杆通道 (原始值 364~1684) */
    uint16_t ch0;       // 右摇杆水平
    uint16_t ch1;       // 右摇杆垂直
    uint16_t ch2;       // 左摇杆垂直
    uint16_t ch3;       // 左摇杆水平
    uint16_t dial;      // 拨轮 (原始值 364~1684)
    /* 开关和按键 */
    uint8_t cns;        // 挡位 (原始值 0=C/上, 1=N/中, 2=S/下)
    uint8_t pause;      // 暂停键
    uint8_t trigger;    // 扳机键
    uint8_t custom_l;   // 自定义键左
    uint8_t custom_r;   // 自定义键右
    /* 鼠标 */
    #pragma pack(1)
    struct {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t l;
        uint8_t r;
    } mouse;
    #pragma pack()
    /* 键盘 */
    #pragma pack(1)
    union {
        uint16_t key_code;
        struct {
            uint16_t W:1;
            uint16_t S:1;
            uint16_t A:1;
            uint16_t D:1;
            uint16_t SHIFT:1;
            uint16_t CTRL:1;
            uint16_t Q:1;
            uint16_t E:1;
            uint16_t R:1;
            uint16_t F:1;
            uint16_t G:1;
            uint16_t Z:1;
            uint16_t X:1;
            uint16_t C:1;
            uint16_t V:1;
            uint16_t B:1;
        } bit;
    } kb;
    #pragma pack()
} vtm_rc_t;
#pragma pack()

extern vtm_rc_t vtm_rc;
void vtm_callback_handler(uint8_t *buff);
void FuseFromVTM(void);


#endif /* __PROT_VTM_H__ */
