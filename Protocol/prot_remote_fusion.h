/**
 * @file prot_input_fusion.h
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
 *   ==================== 使用方式 ====================
 *   
 *   // 1. 在 TIM2 14ms 中断回调中调用融合处理
 *   RemoteFusion_14msTimHandler();
 *   
 *   // 2. 使用按键扫描 API
 *   if (key_scan_clear(KB_Z)) {
 *       // Z 键被按下...
 *   }
 *
 * @author RoboMaster
 * @date 2026
 */

#ifndef __PROT_INPUT_FUSION_H__
#define __PROT_INPUT_FUSION_H__

#include "main.h"
#include "prot_dr16.h"
#include "prot_vtm.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==============================================================================
 *  宏定义 (Macro Definitions)
 * ============================================================================== */

/* 超时时间 (TIM2 tick 计数, 每 tick = 14ms) */
#define REMOTE_TIMEOUT_TICKS    (7)     // 约 100ms (7 * 14ms ≈ 98ms)

/* 丢包率统计周期 (14ms 计数) */
#define PACKET_STAT_PERIOD      (71)    // 约 1 秒 (71 * 14ms ≈ 1s)

/* 摇杆死区阈值 */
#define RC_DEADZONE             (30)

/* 摇杆缩放系数 
 * ch1: 右摇杆水平  ch2: 右摇杆垂直
 * ch3: 左摇杆垂直  ch4: 左摇杆水平
 */
#define RC_CH1_SCALE    0.000006f   // 右摇杆水平
#define RC_CH2_SCALE    12.0f       // 右摇杆垂直
#define RC_CH3_SCALE    10.0f       // 左摇杆垂直
#define RC_CH4_SCALE    10.0f       // 左摇杆水平

/* 摇杆方向判断阈值 */
#define RC_LEVER_THRESHOLD  (500)

/* 按键功能映射 - arm_task */
#define KEY_ARM_WALKING             KB_Q    // 行进
#define KEY_ARM_CUSTOM              KB_E    // 使用自定义控制器
#define KEY_RESET_ALL               KB_R    // 重置所有自动任务的状态
#define KEY_TAKE_SUPPLY_UNIT        KB_B    // 取补给区的能量单元
#define KEY_STORE_UNIT              KB_Z    // 把能量单元放在车上
#define KEY_TAKE_CAR_UNIT           KB_C    // 取车上的能量单元
#define KEY_CALIB_RECORD            KB_F    // 标定记录
#define KEY_WF_START                KB_G    // Workflow开始/恢复
#define KEY_GRIPPER                 KB_V    // 夹爪控制

/* 按键功能映射 - chassis_task */
#define KEY_ROTATE                  KB_X    // 小陀螺
#define KEY_BACK                    KB_B    // 大yaw转180度
#define KEY_POWER_SWITCH            KB_SHIFT// 速度模式切换

/* 摇杆方向判断宏 (使用融合后的数据，向上为正，向左为正) */
#define LEFT_LEVER_LU   (remote_fusion.channel.ch3 >  RC_LEVER_THRESHOLD && remote_fusion.channel.ch4 >  RC_LEVER_THRESHOLD)  // 左摇杆居左上
#define LEFT_LEVER_RU   (remote_fusion.channel.ch3 < -RC_LEVER_THRESHOLD && remote_fusion.channel.ch4 >  RC_LEVER_THRESHOLD)  // 左摇杆居右上
#define LEFT_LEVER_RD   (remote_fusion.channel.ch3 < -RC_LEVER_THRESHOLD && remote_fusion.channel.ch4 < -RC_LEVER_THRESHOLD)  // 左摇杆居右下
#define LEFT_LEVER_LD   (remote_fusion.channel.ch3 >  RC_LEVER_THRESHOLD && remote_fusion.channel.ch4 < -RC_LEVER_THRESHOLD)  // 左摇杆居左下
#define RIGHT_LEVER_LU  (remote_fusion.channel.ch1 >  RC_LEVER_THRESHOLD && remote_fusion.channel.ch2 >  RC_LEVER_THRESHOLD)  // 右摇杆居左上
#define RIGHT_LEVER_RU  (remote_fusion.channel.ch1 < -RC_LEVER_THRESHOLD && remote_fusion.channel.ch2 >  RC_LEVER_THRESHOLD)  // 右摇杆居右上
#define RIGHT_LEVER_RD  (remote_fusion.channel.ch1 < -RC_LEVER_THRESHOLD && remote_fusion.channel.ch2 < -RC_LEVER_THRESHOLD)  // 右摇杆居右下
#define RIGHT_LEVER_LD  (remote_fusion.channel.ch1 >  RC_LEVER_THRESHOLD && remote_fusion.channel.ch2 < -RC_LEVER_THRESHOLD)  // 右摇杆居左下

/* ==============================================================================
 *  枚举定义 (Enum Definitions)
 * ============================================================================== */

///**
// * @brief 键盘按键索引枚举
// * @note  数值对应 key_code 的位偏移，也作为 kb_status[] 的下标
// */
//typedef enum {
//    KB_Q     = 0,
//    KB_E     = 1,
//    KB_R     = 2,
//    KB_F     = 3,
//    KB_G     = 4,
//    KB_Z     = 5,
//    KB_X     = 6,
//    KB_C     = 7,
//    KB_V     = 8,
//    KB_B     = 9,
//    KB_CTRL  = 10,
//    KB_SHIFT = 11,
//    KB_NULL  = 12   // 用户不想使用的按键集
//} key_index_e;

/**
 * @brief VTM物理按键索引枚举
 * @note  用于VTM接收端的4个物理按键
 */
typedef enum {
    VTM_BTN_PAUSE        = 0,   // 暂停按键
    VTM_BTN_TRIGGER      = 1,   // 扳机键
    VTM_BTN_CUSTOM_LEFT  = 2,   // 自定义按键(左)
    VTM_BTN_CUSTOM_RIGHT = 3,   // 自定义按键(右)
    VTM_BTN_COUNT        = 4    // 按键数量
} vtm_btn_index_e;

///**
// * @brief 按键状态枚举
// */
//typedef enum {
//    KEY_END = 0,
//    KEY_RUN = 1
//} rc_key_status_e;

///**
// * @brief 开关/挡位位置枚举 (1=上, 2=下, 3=中)
// */
//typedef enum {
//    RC_UP = 1,      // 上
//    RC_DN = 2,      // 下
//    RC_MI = 3       // 中
//} rc_sw_pos_e;

/**
 * @brief 通道状态枚举
 */
typedef enum {
    REMOTE_CHANNEL_OFFLINE = 0,
    REMOTE_CHANNEL_ONLINE
} RemoteChannelStatus_e;

/**
 * @brief 输入源枚举
 */
typedef enum {
    REMOTE_SOURCE_NONE = 0,
    REMOTE_SOURCE_DR16,     // DR16 遥控器
    REMOTE_SOURCE_VTM       // VTM 图传模块
} RemoteSource_e;

/* ==============================================================================
 *  结构体/联合体定义 (Struct/Union Definitions)
 * ============================================================================== */

/**
 * @brief 融合后的鼠标数据
 */
typedef struct {
    int16_t x;          // X 轴速度
    int16_t y;          // Y 轴速度
    int16_t z;          // 滚轮
    uint8_t left;       // 左键
    uint8_t right;      // 右键
} RemoteFusion_Mouse_t;

/**
 * @brief 融合后的键盘数据 (位域)
 */
typedef union {
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
} RemoteFusion_Keyboard_t;

/**
 * @brief 融合后的摇杆数据 (归一化到 -660 ~ 660)
 * @note  方向约定：向上为正，向左为正
 */
typedef struct {
    int16_t ch1;        // 右摇杆水平 (向左为正)
    int16_t ch2;        // 右摇杆垂直 (向上为正)
    int16_t ch3;        // 左摇杆垂直 (向上为正)
    int16_t ch4;        // 左摇杆水平 (向左为正)
    int16_t dial;       // 拨轮
} RemoteFusion_Channel_t;

/**
 * @brief 融合后的开关数据
 */
typedef struct {
    uint8_t sw1;        // 开关1 (DR16左拨杆 / VTM挡位)
    uint8_t sw2;        // 开关2 (DR16右拨杆)
    uint8_t cns;        // VTM挡位切换开关 CNS (1=上, 2=下, 3=中，跟dr16sw一样)
} RemoteFusion_Switch_t;

/**
 * @brief 通道状态统计数据 (用于 debug 观察)
 */
typedef struct {
    /* 通道在线状态 */
    RemoteChannelStatus_e dr16_status;  // DR16 通道状态
    RemoteChannelStatus_e vtm_status;   // VTM 通道状态
    
    /* TIM2 计数 */
    uint32_t tim2_tick_cnt;     // TIM2 中断计数 (每14ms++)
    
    /* 接收计数 (当前统计周期) */
    uint32_t dr16_rx_cnt;       // DR16 接收计数
    uint32_t vtm_rx_cnt;        // VTM 接收计数
    
    /* 丢包率 (0-100%) */
    float dr16_loss_rate;       // DR16 丢包率
    float vtm_loss_rate;        // VTM 丢包率
} RemoteFusion_Status_t;

/**
 * @brief 融合模块主结构体
 */
typedef struct {
    /* 融合后的数据 */
    RemoteFusion_Channel_t channel;
    RemoteFusion_Switch_t sw;
    RemoteFusion_Mouse_t mouse;
    RemoteFusion_Keyboard_t keyboard;
    
    /* 当前使用通道 */
    RemoteSource_e current_source;
    
    /* 通道状态统计 (用于 debug) */
    RemoteFusion_Status_t status;
} RemoteFusion_Handle_t;

/* ==============================================================================
 *  外部变量声明 (Extern Variables)
 * ============================================================================== */

/* 键盘按键状态数组 */
//extern int kb_status[13];
//extern int kb_last_status[13];

/* VTM按键状态数组 */
extern int vtm_btn_status[VTM_BTN_COUNT];
extern int vtm_btn_last_status[VTM_BTN_COUNT];

/* 融合数据实例 */
extern RemoteFusion_Handle_t remote_fusion;

/* ==============================================================================
 *  函数声明 (Function Declarations)
 * ============================================================================== */

/* -------------------- 键盘按键扫描 -------------------- */

/**
 * @brief  按键扫描，按一次对应状态取反，保留状态
 * @param  key_index: 按键索引 (key_index_e)
 * @return 当前状态 0/1
 */
uint8_t key_scan_save(key_index_e key_index);

/**
 * @brief  按键扫描，检测为TRUE后清除，只起一次作用
 * @param  key_index: 按键索引 (key_index_e)
 * @return 1=按下瞬间, 0=未按下
 */
uint8_t key_scan_clear(key_index_e key_index);

/**
 * @brief  按键松手检测，松手瞬间返回一次 TRUE
 * @param  key_index: 按键索引 (key_index_e)
 * @return 1=松手瞬间, 0=其他状态
 */
uint8_t key_scan_release_once(key_index_e key_index);

/* -------------------- VTM物理按键扫描 -------------------- */

/**
 * @brief  VTM按键扫描，按一次对应状态取反，保留状态
 * @param  btn_index: VTM按键索引 (vtm_btn_index_e)
 * @return 当前状态 0/1
 */
uint8_t vtm_btn_scan_save(vtm_btn_index_e btn_index);

/**
 * @brief  VTM按键扫描，检测为TRUE后清除，只起一次作用
 * @param  btn_index: VTM按键索引 (vtm_btn_index_e)
 * @return 1=按下瞬间, 0=未按下
 */
uint8_t vtm_btn_scan_clear(vtm_btn_index_e btn_index);

/* -------------------- 定时器与通道状态 -------------------- */

/**
 * @brief 14ms 定时器回调处理函数
 * @note  在 TIM2 中断回调中调用，每 14ms 执行一次
 *        完成数据融合、超时检测、丢包率统计
 */
void RemoteFusion_14msTimHandler(void);

/**
 * @brief 设置 DR16 通道在线
 * @note  在 rc_callback_handler 解析成功后调用
 */
void RemoteFusion_SetDR16Online(void);

/**
 * @brief 设置 VTM 通道在线
 * @note  在 vtm_callback_handler 解析成功后调用
 */
void RemoteFusion_SetVTMOnline(void);

#ifdef __cplusplus
}
#endif

#endif /* __PROT_INPUT_FUSION_H__ */

