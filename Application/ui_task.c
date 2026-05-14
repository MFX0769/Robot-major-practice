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
#include "prot_remote_fusion.h"
#include "shoot_task.h"
#include "prot_dm_motor.h"
/* ========== 外部变量声明 ========== */
extern Taurus_imu_data_t     imu_data;
extern robot_status_t        robot_status;
extern game_status_t         Game_status;
extern vision_t              vision;
extern shoot_t               shoot;
extern dm_motor_t 			 dm3507_motor;
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
	
	int8_t pitch_tem = dm3507_motor.t_rotor;
	ui_g_Ungroup_pitch_temperature->number = pitch_tem;
    
    /* ----- 2. 视觉锁定椭圆 ----- */
    // 判断视觉锁定状态：aim_status == AIMING 且 online
    uint8_t color = 8;  // 默认白色
    
    if (vision.shoot_enable == 1) {
        color = 2;      // 绿色：允许发弹
    } 
    else if (vision.aim_status == AIMING) {
        color = 1;      // 黄色：锁定到人
    }
    // else 保持白色
    
    ui_g_Ungroup_vision->color = color;
	
    ui_g_Ungroup_shoot_mode->number = dm3507_motor.err_code;
    
	ui_g_Ungroup_vision_trice_id->number = vision_tx.data.trace_id;
    
	ui_update_g_Ungroup();

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
    
//    // 多次初始化确保客户端收到
//    for (int i = 0; i < UI_INIT_RETRY; i++) {
//        ui_init_g_Ungroup();  // 初始化所有元素（Pitch数值 + Pitch字符串 + 视觉椭圆）
//        osDelay(1);
//    }
	
	ui_init_g_Ungroup();
	
}

/* ========== 主任务 ========== */
uint32_t ui_update_cnt = 0;

/**
  * @brief  UI主任务
  */
void ui_task(void const* argument)
{
    uint32_t thread_wake_time = osKernelSysTick();
    
    // 初始化
    ui_init();
    for(int i = 0; i < 30; i++) {
        thread_wake_time = osKernelSysTick();
        ui_init();
        osDelayUntil(&thread_wake_time, 1);
    }
    for (;;)
    {
        thread_wake_time = osKernelSysTick();
        
        ui_update_cnt++;
        if (Game_status.game_progress == 0 || Game_status.game_progress == 1 || Game_status.game_progress == 5) 
		{
			if(ui_update_cnt % 20 == 0)
				ui_update();
			else
				ui_init();
        } 
		else if(remote_fusion.keyboard.bit.CTRL && remote_fusion.keyboard.bit.Z)
		{
			ui_init();
        }
		else		
		{
			ui_update();
		}
        
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
