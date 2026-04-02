/**
 * @file prot_remote_fusion.c
 * @brief 多通道输入融合模块 - 统一键鼠数据接口
 * @details
 *   本模块融合来自多个通道的键鼠输入：
 *   1. DR16 遥控器 (常规键鼠数据)
 *   2. VTM 图传模块 (VT03/VT13 遥控数据)
 *
 *   优先级策略：DR16 > VTM (DR16在线时优先使用DR16数据)
 *   
 *   注意：本模块只负责数据融合，不负责事件生成。
 *         事件处理在 arm_task 中完成。
 *
 * @author RoboMaster
 * @date 2026
 */

#include "prot_remote_fusion.h"
#include <string.h>
#include <stdint.h>
/* ==================== 全局变量 ==================== */

RemoteFusion_Handle_t remote_fusion = {0};

/* ==================== 超时检测变量 (内部使用) ==================== */
static uint32_t s_dr16_last_update_tick = 0;    // DR16 最后更新时的 tick
static uint32_t s_vtm_last_update_tick = 0;     // VTM 最后更新时的 tick
static uint32_t s_last_stat_tick = 0;           // 上次统计时的 tick

/* ==============================================================================
 *  键盘按键扫描
 * ============================================================================== */

int kb_last_status[13] = { 0 };
//int kb_status[13] = { 0 };

/* VTM 按键状态数组 */
int vtm_btn_status[VTM_BTN_COUNT] = { 0 };
int vtm_btn_last_status[VTM_BTN_COUNT] = { 0 };

///**
// * @brief  按键映射: 从融合后的 key_code 中提取指定按键状态
// * @note   读融合后的数据而非直接读 rc，这样 DR16/VTM 通道通用
// */
//static uint16_t key_map(key_index_e key_index)
//{
//    uint16_t key_code = remote_fusion.keyboard.key_code;
//    switch (key_index) {
//        case KB_Q:     return (key_code >> 6)  & 1;
//        case KB_E:     return (key_code >> 7)  & 1;
//        case KB_R:     return (key_code >> 8)  & 1;
//        case KB_F:     return (key_code >> 9)  & 1;
//        case KB_G:     return (key_code >> 10) & 1;
//        case KB_Z:     return (key_code >> 11) & 1;
//        case KB_X:     return (key_code >> 12) & 1;
//        case KB_C:     return (key_code >> 13) & 1;
//        case KB_V:     return (key_code >> 14) & 1;
//        case KB_B:     return (key_code >> 15) & 1;
//        case KB_CTRL:  return (key_code >> 5)  & 1;
//        case KB_SHIFT: return (key_code >> 4)  & 1;
//        case KB_NULL:  return 0;
//        default:       return 0;
//    }
//}

///**
// * @brief     按键扫描，按一次对应状态取反，保留状态，可一直起作用
// * @param[in] key_index: 按键序号
// * @retval    void
// */
//uint8_t key_scan_save(key_index_e key_index)
//{
//	memcpy(kb_last_status, kb_status, sizeof(kb_status));
//	
//    static uint8_t key_press[13] = {0};
//    uint16_t key = key_map(key_index);  //获取当前按键状态
//    if (key && key_press[key_index] == 0) {  //如果当前按键按下且之前没有按下
//        key_press[key_index] = 1;  //记录上一次状态，防止重复进入
//        kb_status[key_index] = !kb_status[key_index];  //全局状态标志位取反 0-1
//    } else if (key == 0) {
//        key_press[key_index] = 0;  //松开该按键才允许下一次进入
//    }
//    return kb_status[key_index];
//}

////检测为TURE后清除，不保留状态，只起一次作用,其实就等于直接读rc.bit.X是否为1
//uint8_t key_scan_clear(key_index_e key_index)
//{
//    uint8_t res = 0;
//    key_scan_save(key_index);  //如果按下一次按键，则反转标志位
//    if (kb_status[key_index] == KEY_RUN) {
//        kb_status[key_index] = KEY_END;  //清除标志位
//        res = 1;  //返回TRUE
//    } else if (kb_status[key_index] == KEY_END) {
//        res = 0;
//    }
//    return res;
//}

///**
// * @brief     按键松手检测，松手后置1且只起一次作用
// * @param[in] key_index: 按键序号
// * @retval    1=松手瞬间，0=其他状态
// */
//uint8_t key_scan_release_once(key_index_e key_index)
//{
//    static uint8_t key_last_raw[13] = {0};
//    uint8_t res = 0;
//    uint8_t key_now = (uint8_t)key_map(key_index);

//    if ((key_last_raw[key_index] != 0U) && (key_now == 0U)) {
//        res = 1U;
//    }

//    key_last_raw[key_index] = key_now;
//    return res;
//}
/* ==============================================================================
 *  VTM 按键扫描实现 (暂停、扳机、自定义左、自定义右)
 * ============================================================================== */

/**
 * @brief  获取VTM按键当前状态
 * @param  btn_index: VTM按键索引
 * @return 1=按下, 0=未按下
 */
static uint8_t vtm_btn_get_state(vtm_btn_index_e btn_index)
{
    switch (btn_index) {
        case VTM_BTN_PAUSE:        return vtm_rc.pause;
        case VTM_BTN_TRIGGER:      return vtm_rc.trigger;
        case VTM_BTN_CUSTOM_LEFT:  return vtm_rc.custom_l;
        case VTM_BTN_CUSTOM_RIGHT: return vtm_rc.custom_r;
        default: return 0;
    }
}

/**
 * @brief  VTM按键扫描，按一次对应状态取反，保留状态
 */
uint8_t vtm_btn_scan_save(vtm_btn_index_e btn_index)
{
    if (btn_index >= VTM_BTN_COUNT) return 0;
    
    memcpy(vtm_btn_last_status, vtm_btn_status, sizeof(vtm_btn_status));
    
    static uint8_t btn_press_enable[VTM_BTN_COUNT] = {0};
    uint8_t pressed = vtm_btn_get_state(btn_index);
    
    if (pressed && btn_press_enable[btn_index] == 0) {
        btn_press_enable[btn_index] = 1;
        vtm_btn_status[btn_index] = !vtm_btn_status[btn_index];
    } else if (!pressed) {
        btn_press_enable[btn_index] = 0;
    }
    return vtm_btn_status[btn_index];
}

/**
 * @brief  VTM按键扫描，检测为TRUE后清除，只起一次作用
 */
uint8_t vtm_btn_scan_clear(vtm_btn_index_e btn_index)
{
    if (btn_index >= VTM_BTN_COUNT) return 0;
    
    uint8_t res = 0;
    vtm_btn_scan_save(btn_index);
    if (vtm_btn_status[btn_index] == KEY_RUN) {
        vtm_btn_status[btn_index] = KEY_END;
        res = 1;
    } else if (vtm_btn_status[btn_index] == KEY_END) {
        res = 0;
    }
    return res;
}

/* ==================== 私有函数声明 ==================== */

static void FuseChannelData(void);
static void FuseFromDR16(void);
static void FuseFromVTM(void);
static void CalcPacketLossRate(void);

/* ==================== 通道状态设置接口 ==================== */

void RemoteFusion_SetDR16Online(void)
{
    remote_fusion.status.dr16_status = REMOTE_CHANNEL_ONLINE;
    s_dr16_last_update_tick = remote_fusion.status.tim2_tick_cnt;
    remote_fusion.status.dr16_rx_cnt++;
}

void RemoteFusion_SetVTMOnline(void)
{
    remote_fusion.status.vtm_status = REMOTE_CHANNEL_ONLINE;
    s_vtm_last_update_tick = remote_fusion.status.tim2_tick_cnt;
    remote_fusion.status.vtm_rx_cnt++;
}

/* ==================== 14ms 定时器处理 (TIM2 回调) ==================== */

/**
 * @brief 14ms 定时器回调处理函数
 * @note  在 TIM2 中断回调中调用，每 14ms 执行一次
 *        完成数据融合、超时检测、丢包率统计
 */
void RemoteFusion_14msTimHandler(void)
{
    // TIM2 tick 计数
    remote_fusion.status.tim2_tick_cnt++;
    
    // 检查超时 (基于 TIM2 tick 计数)
    if (remote_fusion.status.vtm_status == REMOTE_CHANNEL_ONLINE) {
        if ((remote_fusion.status.tim2_tick_cnt - s_vtm_last_update_tick) > REMOTE_TIMEOUT_TICKS) {
            remote_fusion.status.vtm_status = REMOTE_CHANNEL_OFFLINE;
        }
    }
     if (remote_fusion.status.dr16_status == REMOTE_CHANNEL_ONLINE) {
        if ((remote_fusion.status.tim2_tick_cnt - s_dr16_last_update_tick) > REMOTE_TIMEOUT_TICKS) {
            remote_fusion.status.dr16_status = REMOTE_CHANNEL_OFFLINE;
        }
    }
    
   
    // 融合通道数据
    FuseChannelData();
    
    // 计算丢包率
    CalcPacketLossRate();
}

/* ==================== 私有函数实现 ==================== */
/**
 * @brief 融合各通道数据
 * @details
 *   优先级策略：VTM > DR16
 *   VTM在线时优先使用VTM数据，否则使用DR16数据
 */
static void FuseChannelData(void)
{
    uint8_t dr16_online = (remote_fusion.status.dr16_status == REMOTE_CHANNEL_ONLINE);
    uint8_t vtm_online = (remote_fusion.status.vtm_status == REMOTE_CHANNEL_ONLINE);

    // // 双在线时：键鼠优先 VTM，遥控(摇杆/拨轮/开关)优先 DR16
    // if (dr16_online && vtm_online) {
    //     FuseFromDR16();
    //     remote_fusion.mouse.x = vtm_rc.mouse.x;
    //     remote_fusion.mouse.y = vtm_rc.mouse.y;
    //     remote_fusion.mouse.z = vtm_rc.mouse.z;
    //     remote_fusion.mouse.left = vtm_rc.mouse.l;
    //     remote_fusion.mouse.right = vtm_rc.mouse.r;
    //     remote_fusion.keyboard.key_code = vtm_rc.kb.key_code;
    //     remote_fusion.current_source = REMOTE_SOURCE_DR16;
    //     return;
    // }
    if (vtm_online) {
        FuseFromVTM();
        remote_fusion.current_source = REMOTE_SOURCE_VTM;
        return;
    }
    // 单在线：保持原有优先级，VTM > DR16
    if (dr16_online) {
        FuseFromDR16();
        remote_fusion.current_source = REMOTE_SOURCE_DR16;
        return;
    }


    
    // 没有任何通道在线，保持上一次的数据 (不清零，避免闪烁)
    remote_fusion.current_source = REMOTE_SOURCE_NONE;
}

/**
 * @brief 计算丢包率
 * @note  每 PACKET_STAT_PERIOD 个 14ms 周期统计一次 (约1秒)
 */
static void CalcPacketLossRate(void)
{
    uint32_t elapsed = remote_fusion.status.tim2_tick_cnt - s_last_stat_tick;
    
    if (elapsed >= PACKET_STAT_PERIOD) {
        // DR16 丢包率: (期望次数 - 实际接收次数) / 期望次数
        if (remote_fusion.status.dr16_status == REMOTE_CHANNEL_ONLINE || remote_fusion.status.dr16_rx_cnt > 0) {
            if (elapsed > remote_fusion.status.dr16_rx_cnt) {
                remote_fusion.status.dr16_loss_rate = (float)(elapsed - remote_fusion.status.dr16_rx_cnt) * 100.0f / (float)elapsed;
            } else {
                remote_fusion.status.dr16_loss_rate = 0.0f;
            }
        }
        
        // VTM 丢包率
        if (remote_fusion.status.vtm_status == REMOTE_CHANNEL_ONLINE || remote_fusion.status.vtm_rx_cnt > 0) {
            if (elapsed > remote_fusion.status.vtm_rx_cnt) {
                remote_fusion.status.vtm_loss_rate = (float)(elapsed - remote_fusion.status.vtm_rx_cnt) * 100.0f / (float)elapsed;
            } else {
                remote_fusion.status.vtm_loss_rate = 0.0f;
            }
        }
        
        // 重置计数
        s_last_stat_tick = remote_fusion.status.tim2_tick_cnt;
        remote_fusion.status.dr16_rx_cnt = 0;
        remote_fusion.status.vtm_rx_cnt = 0;
    }
}
/**
 * @brief 从VTM原始数据融合（以 remote_fusion 通道定义为准）
 * @note  在此处完成方向转换和CNS映射
 * @note  统一定义：remote_fusion.ch3=左摇杆水平，remote_fusion.ch4=左摇杆垂直
 * @note  VTM文档通道命名与DR16不同（VTM左摇杆垂直对应其“ch3”，在本工程变量为 vtm_rc.ch2）
 */
static void FuseFromVTM(void)
{
    // 鼠标
    remote_fusion.mouse.x = vtm_rc.mouse.x;
    remote_fusion.mouse.y = vtm_rc.mouse.y;
    remote_fusion.mouse.z = vtm_rc.mouse.z;
    remote_fusion.mouse.left = vtm_rc.mouse.l;
    remote_fusion.mouse.right = vtm_rc.mouse.r;
    
    // 键盘
    remote_fusion.keyboard.key_code = vtm_rc.kb.key_code;
    
    // 摇杆 - 从原始值转换为带符号值
    // 方向约定：向上为正，向左为正
    // 原始值范围: 364~1684, 中值1024
    remote_fusion.channel.ch1 = (int16_t)(1024 - vtm_rc.ch0);   // 右摇杆水平 (向左为正)
    remote_fusion.channel.ch2 = (int16_t)(vtm_rc.ch1 - 1024);   // 右摇杆垂直 (向上为正)
    remote_fusion.channel.ch4 = (int16_t)(vtm_rc.ch2 - 1024);   // remote_fusion.ch4: 左摇杆垂直 (向上为正)
    remote_fusion.channel.ch3 = (int16_t)(1024 - vtm_rc.ch3);   // remote_fusion.ch3: 左摇杆水平 (向左为正)
    remote_fusion.channel.dial = (int16_t)(1024 - vtm_rc.dial); // 拨轮
    
    // 死区处理
    if (remote_fusion.channel.ch1 > -RC_DEADZONE && remote_fusion.channel.ch1 < RC_DEADZONE) remote_fusion.channel.ch1 = 0;
    if (remote_fusion.channel.ch2 > -RC_DEADZONE && remote_fusion.channel.ch2 < RC_DEADZONE) remote_fusion.channel.ch2 = 0;
    if (remote_fusion.channel.ch3 > -RC_DEADZONE && remote_fusion.channel.ch3 < RC_DEADZONE) remote_fusion.channel.ch3 = 0;
    if (remote_fusion.channel.ch4 > -RC_DEADZONE && remote_fusion.channel.ch4 < RC_DEADZONE) remote_fusion.channel.ch4 = 0;
    
    // 开关 - VTM CNS 原始值映射到DR16兼容值
    // 原始值: C=0(上), N=1(中), S=2(下)
    // 目标值: RC_UP=1(上), RC_DN=2(下), RC_MI=3(中)
    switch (vtm_rc.cns) {
        case 0: remote_fusion.sw.cns = RC_UP; break;  // C档(上) -> RC_UP
        case 1: remote_fusion.sw.cns = RC_MI; break;  // N档(中) -> RC_MI
        case 2: remote_fusion.sw.cns = RC_DN; break;  // S档(下) -> RC_DN
        default: remote_fusion.sw.cns = RC_MI; break; // 默认中
    }
    remote_fusion.sw.sw1 = remote_fusion.sw.cns;
    remote_fusion.sw.sw2 = RC_UP;  // 默认上
}

/**
 * @brief 从DR16原始数据融合
 * @note  在此处完成方向转换：向上为正，向左为正
 */
static void FuseFromDR16(void)
{
    // 鼠标
    remote_fusion.mouse.x = rc.mouse.x;
    remote_fusion.mouse.y = rc.mouse.y;
    remote_fusion.mouse.z = rc.mouse.z;
    remote_fusion.mouse.left = rc.mouse.l;
    remote_fusion.mouse.right = rc.mouse.r;
    
    // 键盘
    remote_fusion.keyboard.key_code = rc.kb.key_code;
    
    // 摇杆 - 从原始值转换为带符号值
    // 方向约定：向上为正，向左为正（以 remote_fusion 定义为准：ch3水平/ch4垂直）
    // 原始值范围: 364~1684, 中值1024
    remote_fusion.channel.ch1 = (int16_t)(1024 - rc.ch1);   // 右摇杆水平 (向左为正)
    remote_fusion.channel.ch2 = (int16_t)(rc.ch2 - 1024);   // 右摇杆垂直 (向上为正)
    remote_fusion.channel.ch3 = (int16_t)(1024 - rc.ch3);   // remote_fusion.ch3: 左摇杆水平 (向左为正)
    remote_fusion.channel.ch4 = (int16_t)(rc.ch4 - 1024);   // remote_fusion.ch4: 左摇杆垂直 (向上为正)
    remote_fusion.channel.dial = (int16_t)(rc.ch5 - 1024); // 拨轮
    
    // 死区处理
    if (remote_fusion.channel.ch1 > -RC_DEADZONE && remote_fusion.channel.ch1 < RC_DEADZONE) remote_fusion.channel.ch1 = 0;
    if (remote_fusion.channel.ch2 > -RC_DEADZONE && remote_fusion.channel.ch2 < RC_DEADZONE) remote_fusion.channel.ch2 = 0;
    if (remote_fusion.channel.ch3 > -RC_DEADZONE && remote_fusion.channel.ch3 < RC_DEADZONE) remote_fusion.channel.ch3 = 0;
    if (remote_fusion.channel.ch4 > -RC_DEADZONE && remote_fusion.channel.ch4 < RC_DEADZONE) remote_fusion.channel.ch4 = 0;
    
    // 开关 - DR16 sw 已经是 1=上, 2=下, 3=中
    remote_fusion.sw.sw1 = rc.sw1;
    remote_fusion.sw.sw2 = rc.sw2;
    remote_fusion.sw.cns = rc.sw1;  // DR16 使用 sw1 作为 CNS
}



