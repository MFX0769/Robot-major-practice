/**
  * @file    chassis_task.c
  * @brief   底盘控制任务 (麦克纳姆轮)
  * @note    参考 RoboticMajor_Pratice_code
  */

#include "chassis_task.h"
#include "pid.h"
#include "mode_switch_task.h"
#include "control_def.h"
#include "prot_rm_motor.h"
#include "can_comm.h"
#include "cmsis_os.h"
#include "dbg_print.h"
#include "config.h"
#include "prot_sbus.h"
#include <string.h>

/* 外部电机变量 */
extern dji_motor_t chassis_2006[4];

/* 底盘电机结构体 */
chassis_motor_t chassis_motor;

/* 底盘电机速度环PID */
/* TODO(2026-06-23): 已声明+初始化，但未接入计算——当前 chassis_mecanum_resolve()
 * 算出的速度目标直接当 tx_current 发送，没有闭环。等IMU/底盘联调有空再接上
 * （需要电机实际转速反馈 -> PID_calc -> tx_current）。 */
static pid_t chassis_motor_pid[4];

/* 线位跟踪模式 - 转向PID */
static pid_t line_track_pid;

/**
  * @brief  底盘参数初始化
  */
static void chassis_init(void)
{
    memset(&chassis_motor, 0, sizeof(chassis_motor_t));
    
    /* 4个电机速度环PID初始化 */
    for(int i = 0; i < 4; i++)
    {
        PID_struct_init(&chassis_motor_pid[i], 500, 100.0f, 5.0f, 0.0f, 1.0f, 0, 0, 0, NONE_IMPROVE);
    }
    
    /* 线位跟踪PID初始化 */
    PID_struct_init(&line_track_pid, 300, 50.0f, 0.5f, 0.0f, 0.0f, 0, 0, 0, NONE_IMPROVE);
}

/**
  * @brief  麦克纳姆轮运动学解算
  * @param  vx: 前进速度
  * @param  vy: 左右速度
  * @param  wz: 旋转角速度
  * @param  L: 底盘轴距系数
  */
static void chassis_mecanum_resolve(float vx, float vy, float wz, float L)
{
    /*
    底盘坐标系：
    FL --- FR
    |       |
    BL --- BR
    
    麦克纳姆轮解算：
    */
    chassis_motor.motor[0] = (int16_t)(vx + vy + L * wz);    /* 前左 FL */
    chassis_motor.motor[1] = (int16_t)(vx - vy - L * wz);    /* 前右 FR */
    chassis_motor.motor[2] = (int16_t)(vx - vy + L * wz);    /* 后左 BL */
    chassis_motor.motor[3] = (int16_t)(vx + vy - L * wz);    /* 后右 BR */
}

/**
  * @brief  底盘控制任务
  */
void chassis_task(void const *argu)
{
    uint32_t mode_wake_time = osKernelSysTick();
    static uint32_t print_cnt = 0;
    chassis_init();
#if DEBUG_TEXT_ENABLE
    dbg_print("[CHASSIS] Task started\r\n");
#endif
    
    for(;;)
    {
        mode_wake_time = osKernelSysTick();
        float vx_ref = 0.0f;
        float vy_ref = 0.0f;
        float wz_ref = 0.0f;

        taskENTER_CRITICAL();
        
        switch(ctrl_mode)
        {
            case PROTECT_MODE:
            {
                /* 保护模式 - 清零电机输出 */
                for(int i = 0; i < 4; i++)
                {
                    chassis_motor.motor[i] = 0;
                }
            }break;
            
            case REMOTER_MODE:
            {
                /* SBUS直接驱动，参考RoboticMajor_Pratice_code的rc.ch1/2/3映射 */
                vx_ref = wbus_rc.data.ch3 * CHASSIS_SCALE_VX;
                vy_ref = wbus_rc.data.ch2 * CHASSIS_SCALE_VY;
                wz_ref = wbus_rc.data.ch1 * CHASSIS_SCALE_WZ;
            }break;

            case KEYBOARD_MODE:
            {
                /* 键盘模式 - 暂时和遥控器一样 */
                vx_ref = 0.0f;
                vy_ref = 0.0f;
                wz_ref = 0.0f;
            }break;
            
            case VISION_MODE:
            {
                /* 视觉模式 - TODO */
                vx_ref = 0.0f;
                vy_ref = 0.0f;
                wz_ref = 0.0f;
            }break;
            default:break;
        }
                
        /* 运动学解算 */
        chassis_mecanum_resolve(vx_ref, vy_ref, wz_ref, MECANUM_L);
        
        /* 赋值到电机结构体 (注意：左右顺序可能需要调整) */
        chassis_2006[0].tx_current = chassis_motor.motor[0];  /* 左前 */
        chassis_2006[1].tx_current = chassis_motor.motor[2];  /* 左后 */
        chassis_2006[2].tx_current = chassis_motor.motor[1];  /* 右前 */
        chassis_2006[3].tx_current = chassis_motor.motor[3];  /* 右后 */
        
        taskEXIT_CRITICAL();
        
        /* 通过队列发送电机电流 */
        chassis_can_send();
        
        /* 每500次打印一次状态 */
#if DEBUG_TEXT_ENABLE
        if (++print_cnt >= 500) {
            print_cnt = 0;
            dbg_print("[CHASSIS] mode=%d lock=%d sw1=%d ch1=%d ch2=%d ch3=%d ch4=%d rx[3]=%d rpm=%d\r\n",
                     ctrl_mode, lock_flag, wbus_rc.data.sw1,
                     wbus_rc.data.ch1, wbus_rc.data.ch2, wbus_rc.data.ch3, wbus_rc.data.ch4,
                     chassis_2006[3].receive_cnt, chassis_2006[3].speed_rpm);
        }
#endif
        
        osDelayUntil(&mode_wake_time, CHASSIS_PERIOD);
    }
}
