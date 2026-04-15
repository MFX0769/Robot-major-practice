/**
  * @file    ui_task.c
  * @brief   无人机UI任务，使用RM UI Designer静态版生成代码
  * @note    替换原 judge_send_task
  */

#include "ui_task.h"
#include "cmsis_os.h"
#include "ui_interface.h"
#include "ui_types.h"
#include "ui.h"

#include "prot_T_imu.h"       // 陀螺仪数据
#include "prot_vision.h"      // 视觉数据
#include "prot_rm_judge.h"    // 裁判系统数据

/* ========== 外部变量声明 ========== */
extern Taurus_imu_data_t     imu_data;
extern robot_status_t        robot_status;
extern game_status_t         Game_status;
extern vision_t              vision;

/* ========== 配置 ========== */
#define UI_INIT_RETRY   30
#define UI_TASK_PERIOD  10  // 10ms = 100Hz

/* ========== UI更新函数 ========== */

/**
  * @brief  更新UI：Pitch数值 + 视觉锁定椭圆
  */
static void ui_update(void)
{
    /* ----- 1. Pitch角度数值 ----- */
    float pitch_deg = imu_data.pitch;

    // 放大100倍转为整数（30.5° → 3050）
    ui_g_Ungroup_Pitch_Number->number = (int32_t)(pitch_deg * 100.0f);
    
    /* ----- 2. 视觉锁定椭圆 ----- */
    // 判断视觉锁定状态：aim_status == AIMING 且 online
    uint8_t vision_locked = 0;
    
    if (vision.online && vision.aim_status == AIMING && vision.trace_id != 0) {
        vision_locked = 1;
    }
    
    // 颜色：锁定=绿色(2)，未锁定=红色/其他（根据你的设计，0是主色/红色）
    ui_g_Ungroup_vision->color = vision_locked ? 2 : 0;
    
    // 可选：根据目标位置调整椭圆中心（如果视觉有目标坐标）
    // 注意：你的vision结构体没有直接的target_x/y，需要从其他字段计算或添加
    // 如果有需要，可以在这里更新 start_x/start_y
}

/* ========== 初始化 ========== */

/**
  * @brief  UI初始化
  */
static void ui_init(void)
{
    // 等待裁判系统数据就绪
    while (robot_status.robot_id == 0) {
        osDelay(100);
    }
    
    // 设置机器人ID
    ui_self_id = robot_status.robot_id;
    
    // 多次初始化确保客户端收到
    for (int i = 0; i < UI_INIT_RETRY; i++) {
        ui_init_g_Ungroup();  // 初始化所有元素（Pitch数值 + Pitch字符串 + 视觉椭圆）
        osDelay(1);
    }
	
//	ui_init_g_Ungroup();
	
}

/* ========== 主任务 ========== */

/**
  * @brief  UI主任务
  */
void ui_task(void const* argument)
{
    uint32_t thread_wake_time = osKernelSysTick();
    
    // 初始化
    ui_init();
    
    for (;;)
    {
        thread_wake_time = osKernelSysTick();
        
        // 更新数据
        ui_update();
        
        // 发送更新（静态版：整组发送）
        ui_update_g_Ungroup();
        
        osDelayUntil(&thread_wake_time, UI_TASK_PERIOD);
    }
}

/**
  * @brief  外部调用的手动刷新接口
  */
void ui_manual_init(void)
{
    ui_init_g_Ungroup();
}
