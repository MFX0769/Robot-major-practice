/**
  * @file    chassis_task.c
  * @brief   底盘控制任务 (麦克纳姆�?
  * @note    参�?RoboticMajor_Pratice_code
  */

#include "chassis_task.h"
#include "pid.h"
#include "mode_switch_task.h"
#include "control_def.h"
#include "prot_rm_motor.h"
#include "can_comm.h"
#include "cmsis_os.h"
#include "config.h"
#include "prot_sbus.h"
#include "prot_8irtraking.h"
#include <string.h>

/* 外部电机变量 */
extern dji_motor_t chassis_2006[4];

/* 底盘电机结构�?*/
chassis_motor_t chassis_motor;
extern volatile uint8_t arm_busy;

/* 底盘电机速度环PID，按chassis_2006[]的下�?左前/左后/右前/右后) */
static pid_t chassis_motor_pid[4];

/* 线位跟踪模式 - 转向PID */
static pid_t line_track_pid;

/**
  * @brief  底盘参数初始�?  */
static void chassis_init(void)
{
    memset(&chassis_motor, 0, sizeof(chassis_motor_t));
    
    /* 4个电机速度环PID初始�?*/
    for(int i = 0; i < 4; i++)
    {
        PID_struct_init(&chassis_motor_pid[i], 8000, 1600.0f, 5.0f, 0.0f, 1.0f, 0, 0, 0, NONE_IMPROVE);
    }
    
    /* 线位跟踪PID初始化：maxout=1500跟遥控满转量级接�?ch4满偏×CHASSIS_SCALE_WZ�?640)�?       kp=-15配合position最大�?00，kp*error正好打满maxout；负号是实测确认过的纠偏方向�?       原来外部乘的-150系数已经去掉，方向信息挪到这�?*/
    PID_struct_init(&line_track_pid, 150000, 150.0f, -50.0f, 0.0f, 0.0f, 0, 0, 0, NONE_IMPROVE);
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
    chassis_init();

    /* 巡线模块改用纯GPIO读数字量，不再走UART协议 */
    prot_8irtraking_init();

    for(;;)
    {
        mode_wake_time = osKernelSysTick();

        prot_8irtraking_poll_gpio();

        float vx_ref = 0.0f;
        float vy_ref = 0.0f;
        float wz_ref = 0.0f;
        const prot_8irtraking_data_t *line_data = prot_8irtraking_get_data();

        /* 底盘专用模式：由第二个拨杆sw2(ch7)直接驱动，跟全局ctrl_mode是独立的两个概念 */
        static chassis_mode_e prev_mode = CHASSIS_STOP;
        chassis_mode_e chassis_mode;
        switch (wbus_rc.data.sw2)
        {
            case 2:  chassis_mode = CHASSIS_REMOTER;    break;
            case 0:  chassis_mode = CHASSIS_LINE_TRACK; break;
            default: chassis_mode = CHASSIS_STOP;       break;
        }
        if (chassis_mode != prev_mode) {
            prev_mode = chassis_mode;
        }

        taskENTER_CRITICAL();

        switch(chassis_mode)
        {
            case CHASSIS_STOP:
            {
                /* 静止 - 给机械臂调试用，不需要底盘动 */
                vx_ref = 0.0f;
                vy_ref = 0.0f;
                wz_ref = 0.0f;
            }break;

            case CHASSIS_REMOTER:
            {
                if (arm_busy) {
                    vx_ref = vy_ref = wz_ref = 0.0f;
                } else if (wbus_rc.data.sw1 == 0 && wbus_rc.data.sw3) {
                    vx_ref = vy_ref = wz_ref = 0.0f;
                } else {
                    vx_ref = -wbus_rc.data.ch2 * CHASSIS_SCALE_VX;
                    vy_ref = wbus_rc.data.ch1 * CHASSIS_SCALE_VY;
                    wz_ref = wbus_rc.data.ch4 * CHASSIS_SCALE_WZ;
                }
            }break;

            case CHASSIS_LINE_TRACK:
            {
                vx_ref = 1200.0f;
                vy_ref = 0.0f;

                static uint8_t was_on = 0;
                static uint8_t debounce = 0;
                static uint16_t track_time = 0;
                if (track_time < 0xFFFF) track_time++;
                if (line_data != NULL && line_data->state && track_time > 1000)
                {
                    uint8_t cnt_on_black = 0;
                    for (uint8_t i = 0; i < 8; i++)
                        if (line_data->digital[i]) cnt_on_black++;
                    if (!was_on) {
                        if (cnt_on_black >= 3) { debounce++; if (debounce >= 15) { was_on = 1; debounce = 0; } }
                        else debounce = 0;
                    } else {
                        if (cnt_on_black <= 1) was_on = 0;
                    }
                    if (was_on) {
                        vx_ref = 350.0f;
                        wz_ref = -900.0f;
                    } else {
                        wz_ref = pid_calc(&line_track_pid, (float)line_data->position, 0.0f);
                    }
                }
                /* 神之一手：巡线中始终有�?*/
                wz_ref += wbus_rc.data.ch1 * CHASSIS_SCALE_WZ;
            }break;
            default:break;
        }

        /* arm_busy��������ģʽ */
        if (arm_busy) vx_ref = vy_ref = wz_ref = 0.0f;

        /* 运动学解�?*/
        chassis_mecanum_resolve(vx_ref, vy_ref, wz_ref, MECANUM_L);

        /* chassis_2006[]下标(左前/左后/右前/右后) 对应的运动学目标
           右边两个轮子(右前/右后)需要反�?*/
        float target_rpm[4] = {
            (float)chassis_motor.motor[0],   /* 左前 */
            (float)chassis_motor.motor[2],   /* 左后 */
            -(float)chassis_motor.motor[1],  /* 右前 (反向) */
            -(float)chassis_motor.motor[3],  /* 右后 (反向) */
        };

        /* 速度环PID闭环，最后强制保护：底盘静止或全局保护模式，无条件�?，不管PID算出什�?*/
        uint8_t force_zero = (chassis_mode == CHASSIS_STOP) || (ctrl_mode == PROTECT_MODE);
        for (int i = 0; i < 4; i++)
        {
            float pid_out = pid_calc(&chassis_motor_pid[i], chassis_2006[i].speed_rpm, target_rpm[i]);
            if (force_zero)
            {
                chassis_2006[i].tx_current = 0;
            }
            else
            {
                chassis_2006[i].tx_current = (int16_t)pid_out;
            }
        }

        taskEXIT_CRITICAL();
        
        /* 通过队列发送电机电�?*/
        static uint8_t skip = 0;
        if (++skip % 4 == 0)
            chassis_can_send();
        
        
        osDelayUntil(&mode_wake_time, CHASSIS_PERIOD);
    }
}
