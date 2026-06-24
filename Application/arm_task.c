/**
  * @file    arm_task.c
  * @brief   机械臂主任务实现
  * @note    状态机 + yaw ECD闭环PID + PWM舵机 + 视觉输入
  *          yaw用GM6020，肩/肘/夹爪用PWM舵机
  */

/* Includes ------------------------------------------------------------------*/
#include "arm_task.h"
#include "prot_servo.h"
#include "bsp_pwm.h"
#include "bsp_gpio.h"
#include "prot_sbus.h"
#include "mode_switch_task.h"
#include "control_def.h"
#include "prot_rm_motor.h"
#include "prot_vision.h"
#include "pid.h"
#include "math_calcu.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include <math.h>

/* Private variables ---------------------------------------------------------*/
arm_ctrl_t arm_ctrl;
float test_angle;
/* Private function prototypes -----------------------------------------------*/
static void arm_task_main_loop(void);

/* ========================= Yaw PID初始化 ========================= */
void arm_yaw_pid_init(void)
{
    /* 位置环: fdb=ecd转角度(度), ref=目标角度(度)
     * MaxOutput=500 (喂给速度环), IntegralLimit=200
     * Kp=40.0, Ki=0, Kd=5.0 (参考gimbal_task.c yaw_angle) */
    PID_struct_init(&arm_ctrl.yaw.angle_pid, 500, 200, 40.0f, 0.0f, 5.0f,
                    0, 0, 0, NONE_IMPROVE);

    /* 速度环: fdb=speed_rpm, ref=位置环输出
     * MaxOutput=25000 (6020电压模式), IntegralLimit=10000
     * Kp=198.0, Ki=0, Kd=13.0 (参考gimbal_task.c yaw_spd) */
    PID_struct_init(&arm_ctrl.yaw.spd_pid, 25000, 10000, 198.0f, 0.0f, 13.0f,
                    0, 0, 0, NONE_IMPROVE);

    arm_ctrl.yaw.current_limit = ARM_YAW_CURRENT_LIMIT;
    arm_ctrl.yaw.angle_ref = 0.0f;
    arm_ctrl.yaw.angle_fdb = 0.0f;
    arm_ctrl.yaw.spd_ref = 0.0f;
    arm_ctrl.yaw.spd_fdb = 0.0f;
    arm_ctrl.yaw.output = 0.0f;
}

/* ========================= Yaw PID计算 ========================= */
void arm_yaw_pid_calc(void)
{
    /* --- 双重保险①：保护模式下目标跟随反馈，error=0，PID不会累积/不会主动输出 --- */
    if (ctrl_mode == PROTECT_MODE) {
        arm_ctrl.target.yaw = (float)arm_yaw_motor.ecd * ECD_TO_DEG;
    }

    /* --- 位置环 --- */
    /* fdb: ecd转角度 (0~8192 -> 0~360度) */
    arm_ctrl.yaw.angle_fdb = (float)arm_yaw_motor.ecd * ECD_TO_DEG;

    /* ref: 来自arm_ctrl.target.yaw (-90~90度) */
    arm_ctrl.yaw.angle_ref = arm_ctrl.target.yaw;

    /* 圆周误差 (处理0/360边界) */
    float yaw_err = circle_error(arm_ctrl.yaw.angle_ref, arm_ctrl.yaw.angle_fdb, 360);

    /* 位置环输出 -> 速度环ref */
    arm_ctrl.yaw.spd_ref = pid_calc(&arm_ctrl.yaw.angle_pid,
                                     arm_ctrl.yaw.angle_fdb,
                                     arm_ctrl.yaw.angle_fdb + yaw_err);

    /* --- 速度环 --- */
    /* fdb: speed_rpm (6020反馈) */
    arm_ctrl.yaw.spd_fdb = (float)arm_yaw_motor.speed_rpm;

    /* 速度环输出 -> 电流 */
    arm_ctrl.yaw.output = pid_calc(&arm_ctrl.yaw.spd_pid,
                                    arm_ctrl.yaw.spd_fdb,
                                    arm_ctrl.yaw.spd_ref);

    /* --- 电流限幅 --- */
    if (arm_ctrl.yaw.output > arm_ctrl.yaw.current_limit)
        arm_ctrl.yaw.output = arm_ctrl.yaw.current_limit;
    if (arm_ctrl.yaw.output < -arm_ctrl.yaw.current_limit)
        arm_ctrl.yaw.output = -arm_ctrl.yaw.current_limit;

    /* --- ECD硬限位回拉 (参考gimbal_task.c的YAW_ECD_LEFT/RIGHT模式，超出限位时用回拉力覆盖PID输出) --- */
    if (arm_yaw_motor.ecd >= ARM_YAW_ECD_RIGHT) {
        arm_ctrl.yaw.output = -(float)(arm_yaw_motor.ecd - ARM_YAW_ECD_RIGHT) * ARM_YAW_LIMIT_KP;
    } else if (arm_yaw_motor.ecd <= ARM_YAW_ECD_LEFT) {
        arm_ctrl.yaw.output = (float)(ARM_YAW_ECD_LEFT - arm_yaw_motor.ecd) * ARM_YAW_LIMIT_KP;
    }

    /* --- 双重保险②：发送前再强制清零，不管前面算出什么 --- */
    if (ctrl_mode == PROTECT_MODE) {
        arm_ctrl.yaw.output = 0.0f;
    }

    /* --- 输出到电机 --- */
    arm_yaw_motor.tx_current = (int16_t)arm_ctrl.yaw.output;
    dji_send_current(&arm_yaw_motor, NULL, NULL, NULL);
}

/* ========================= 机械臂任务初始化 ========================= */
void arm_task_init(void)
{
    /* 初始化IK模块 */
    arm_ik_init();

    /* 初始化舵机 */
    servo_init();

    /* 启动PWM输出 */
    bsp_pwm_start();

    /* 初始化控制结构体 */
    arm_ctrl.state = ARM_IDLE;
    arm_ctrl.target.yaw = 0.0f;
    arm_ctrl.target.shoulder = 0.0f;
    arm_ctrl.target.elbow = 0.0f;
    arm_ctrl.target.gripper = 0.0f;
    arm_ctrl.current = arm_ctrl.target;
    arm_ctrl.pos_valid = 0;
    arm_ctrl.vision_valid = 0;
    arm_ctrl.smooth_step = ARM_SMOOTH_STEP_DEBUG;  /* 默认调试模式 */

    /* 舵机供电默认关闭，由SWD(sw3)拨杆每周期控制，不在初始化时无条件使能 */
    bsp_gpio_servo_power_off();

    /* 初始化yaw PID */
    arm_yaw_pid_init();

    /* 清零yaw电机输出 */
    arm_yaw_motor.tx_current = 0;
    dji_send_current(&arm_yaw_motor, NULL, NULL, NULL);

    ARM_DBG("\r\n=== ARM Task Init ===\r\n");
    ARM_DBG("State: IDLE\r\n");
    ARM_DBG("Servos: enabled (shoulder=PA0, elbow=PA1, gripper=PA6)\r\n");
    ARM_DBG("Yaw: GM6020 on CAN2 0x206\r\n");
    ARM_DBG("Smooth step: %.2f deg/cycle (DEBUG mode)\r\n", arm_ctrl.smooth_step);

#if ARM_IK_SELFTEST_EN
    arm_ik_self_test();
#endif
}

/* ========================= 视觉xy -> yaw方向 ========================= */
void arm_vision_update(void)
{
    /* 从vision_fruit读取视觉数据 (只读，不修改协议层) */
    if (!vision_fruit.flag || vision_fruit.rec_cnt == 0) {
        return;
    }

    /* 将像素坐标转换为有符号整数 */
    int16_t pix_x = (int16_t)vision_fruit.x_dis;
    int16_t pix_y = (int16_t)vision_fruit.y_dis;

    /* 限幅 (相机320x240) */
    if (pix_x > 160)  pix_x = 160;
    if (pix_x < -160) pix_x = -160;
    if (pix_y > 120)  pix_y = 120;
    if (pix_y < -120) pix_y = -120;

    /* xy -> yaw方向角 */
    /* atan2(y, x) 返回 -PI~PI，转为角度 */
    float yaw_dir = atan2f((float)pix_y, (float)pix_x) * 180.0f / PI;

    /* 设置yaw目标 */
    arm_set_target_yaw(yaw_dir);

    /* 没有距离数据，shoulder/elbow保持当前位置 */
    /* TODO: 后续接入距离数据后，用IK计算完整目标 */

    ARM_DBG("[VISION] pix=(%d,%d) yaw_dir=%.1f\r\n", pix_x, pix_y, yaw_dir);

    /* 清除计数器，避免重复处理 */
    vision_fruit.rec_cnt = 0;
}

/* ========================= 设置目标位置 ========================= */
void arm_set_target_pos(float x, float y, float z)
{
    arm_ctrl.target_pos.x = x;
    arm_ctrl.target_pos.y = y;
    arm_ctrl.target_pos.z = z;
    arm_ctrl.pos_valid = 1;
    arm_ctrl.state = ARM_MOVING;

    ARM_DBG("[ARM] Target pos: (%.1f, %.1f, %.1f)\r\n", x, y, z);
}

/* ========================= 设置yaw目标角度 ========================= */
void arm_set_target_yaw(float yaw_deg)
{
    /* 限幅 */
    if (yaw_deg > ARM_YAW_MAX) yaw_deg = ARM_YAW_MAX;
    if (yaw_deg < ARM_YAW_MIN) yaw_deg = ARM_YAW_MIN;

    arm_ctrl.target.yaw = yaw_deg;
    arm_ctrl.state = ARM_MOVING;

    ARM_DBG("[ARM] Target yaw: %.1f\r\n", yaw_deg);
}

/* ========================= 切换平滑模式 ========================= */
void arm_set_smooth_mode(arm_smooth_mode_e mode)
{
    switch (mode) {
    case ARM_SMOOTH_DEBUG:
        arm_ctrl.smooth_step = ARM_SMOOTH_STEP_DEBUG;
        ARM_DBG("[ARM] Smooth mode: DEBUG (0.01 deg/cycle)\r\n");
        break;
    case ARM_SMOOTH_NORMAL:
        arm_ctrl.smooth_step = ARM_SMOOTH_STEP_NORMAL;
        ARM_DBG("[ARM] Smooth mode: NORMAL (1.0 deg/cycle)\r\n");
        break;
    }
}

/* ========================= 状态查询/设置 ========================= */
arm_state_e arm_get_state(void)
{
    return arm_ctrl.state;
}

void arm_set_state(arm_state_e state)
{
    static const char *state_names[] = {
        "IDLE", "READY", "MOVING", "GRIP", "RELEASE", "HOME", "ERROR"
    };
    ARM_DBG("[ARM] State: %s -> %s\r\n", state_names[arm_ctrl.state], state_names[state]);
    arm_ctrl.state = state;
}

/* ========================= 调试信息输出 ========================= */
void arm_debug_info(void)
{
    static uint32_t dbg_cnt = 0;
    if (dbg_cnt++ % 200 == 0) {
        /* 状态 */
        ARM_DBG("[ARM] state=%d target=(y=%.1f s=%.1f e=%.1f g=%.1f)\r\n",
                arm_ctrl.state,
                arm_ctrl.target.yaw, arm_ctrl.target.shoulder,
                arm_ctrl.target.elbow, arm_ctrl.target.gripper);
        /* 当前 */
        ARM_DBG("[ARM] cur=(y=%.1f s=%.1f e=%.1f g=%.1f)\r\n",
                arm_ctrl.current.yaw, arm_ctrl.current.shoulder,
                arm_ctrl.current.elbow, arm_ctrl.current.gripper);
        /* Yaw PID */
        ARM_DBG("[ARM] yaw_pid: ref=%.1f fdb=%.1f spd_ref=%.1f spd_fdb=%.1f out=%.0f\r\n",
                arm_ctrl.yaw.angle_ref, arm_ctrl.yaw.angle_fdb,
                arm_ctrl.yaw.spd_ref, arm_ctrl.yaw.spd_fdb,
                arm_ctrl.yaw.output);
    }
}

/* ========================= 机械臂主任务 ========================= */
void arm_task(void)
{
    /* 初始化 */
    arm_task_init();

    /* 任务主循环 */
    for (;;) {
        /* ---------- 舵机供电继电器：只有全局ctrl_mode==REMOTER_MODE时才听SWD(sw3)，其他模式强制断电 ---------- */
        if (ctrl_mode == REMOTER_MODE && wbus_rc.data.sw3)
        {
            bsp_gpio_servo_power_on();
        }
        else
        {
            bsp_gpio_servo_power_off();
        }

        /* ---------- 标定测试模式：SWD(sw3)打上时，ch3(左摇杆竖)直接映射肩关节角度，方便watch读数标定位置 ----------
           跳过正常状态机/yaw PID，只测这一个舵机；sw3松开恢复正常逻辑。
           ch3=0时角度=0(当前位置)；±90是占位缩放系数，实际限幅待用户实测后再给 */
        if (ctrl_mode == REMOTER_MODE && wbus_rc.data.sw3)
        {
             test_angle = ((float)wbus_rc.data.ch3 / 660.0f) * 90.0f;
            if (test_angle > 90.0f)  test_angle = 90.0f;
            if (test_angle < -90.0f) test_angle = -90.0f;
            servo_set_angle(SERVO_SHOULDER, test_angle);
            ARM_DBG("[ARM TEST] ch3=%d shoulder_angle=%.1f\r\n", wbus_rc.data.ch3, test_angle);
            osDelay(ARM_TASK_PERIOD);
            continue;
        }

        /* ---------- 视觉输入更新 ---------- */
        arm_vision_update();

        /* ---------- 状态机 ---------- */
        switch (arm_ctrl.state) {
        case ARM_IDLE:
            /* 空闲，等待指令 */
            break;

        case ARM_READY:
            /* 就绪，可接受目标 */
            break;

        case ARM_MOVING:
            /* 运动状态 */
            if (arm_ctrl.pos_valid) {
                /* 有视觉目标，执行IK */
                int8_t ret = arm_inverse_kinematic(&arm_ctrl.target_pos, &arm_ctrl.target);
                if (ret != 0) {
                    /* IK失败，保持当前位置 */
                    arm_ctrl.target = arm_ctrl.current;
                    ARM_DBG("[ARM] IK FAIL!\r\n");
                }
                arm_ctrl.pos_valid = 0;
            }

            /* 关节限位 */
            arm_joint_limit(&arm_ctrl.target);

            /* 平滑 */
            arm_joint_smooth(&arm_ctrl.target, &arm_ctrl.current, arm_ctrl.smooth_step);

            /* 检查是否到达 */
            {
                float diff_yaw = fabsf(arm_ctrl.target.yaw - arm_ctrl.current.yaw);
                float diff_sh = fabsf(arm_ctrl.target.shoulder - arm_ctrl.current.shoulder);
                float diff_el = fabsf(arm_ctrl.target.elbow - arm_ctrl.current.elbow);

                if (diff_yaw < 0.5f && diff_sh < 0.5f && diff_el < 0.5f) {
                    arm_ctrl.state = ARM_READY;
                    ARM_DBG("[ARM] Reached target!\r\n");
                }
            }
            break;

        case ARM_GRIP:
            arm_ctrl.target.gripper = ARM_GRIPPER_MAX;
            arm_joint_limit(&arm_ctrl.target);
            arm_joint_smooth(&arm_ctrl.target, &arm_ctrl.current, arm_ctrl.smooth_step);
            if (fabsf(arm_ctrl.current.gripper - ARM_GRIPPER_MAX) < 1.0f) {
                arm_ctrl.state = ARM_READY;
            }
            break;

        case ARM_RELEASE:
            arm_ctrl.target.gripper = ARM_GRIPPER_MIN;
            arm_joint_limit(&arm_ctrl.target);
            arm_joint_smooth(&arm_ctrl.target, &arm_ctrl.current, arm_ctrl.smooth_step);
            if (fabsf(arm_ctrl.current.gripper - ARM_GRIPPER_MIN) < 1.0f) {
                arm_ctrl.state = ARM_READY;
            }
            break;

        case ARM_HOME:
            arm_ctrl.target.yaw = 0.0f;
            arm_ctrl.target.shoulder = 0.0f;
            arm_ctrl.target.elbow = 0.0f;
            arm_ctrl.target.gripper = 0.0f;
            arm_joint_limit(&arm_ctrl.target);
            arm_joint_smooth(&arm_ctrl.target, &arm_ctrl.current, arm_ctrl.smooth_step);
            if (fabsf(arm_ctrl.current.yaw) < 0.5f &&
                fabsf(arm_ctrl.current.shoulder) < 0.5f &&
                fabsf(arm_ctrl.current.elbow) < 0.5f) {
                arm_ctrl.state = ARM_IDLE;
            }
            break;

        case ARM_ERROR:
            /* 错误状态，舵机断电 */
            bsp_gpio_servo_power_off();
            /* yaw电机清零 */
            arm_yaw_motor.tx_current = 0;
            dji_send_current(&arm_yaw_motor, NULL, NULL, NULL);
            break;
        }

        /* ---------- 输出到舵机 ---------- */
        if (arm_ctrl.state != ARM_ERROR) {
            servo_set_angle(SERVO_SHOULDER, arm_ctrl.current.shoulder);
            servo_set_angle(SERVO_ELBOW, arm_ctrl.current.elbow);
            servo_set_angle(SERVO_GRIPPER, arm_ctrl.current.gripper);
        }

        /* ---------- Yaw PID计算 (每周期，PROTECT_MODE的双重保险在函数内部) ---------- */
        if (arm_ctrl.state != ARM_ERROR) {
            arm_yaw_pid_calc();
        }

        /* ---------- 调试信息 ---------- */
        arm_debug_info();

        /* ---------- 周期延时 ---------- */
        osDelay(ARM_TASK_PERIOD);
    }
}


/**
  * @brief  视觉目标回调占位实现 (未接入具体逻辑, 仅用于满足链接)
  * @note   TODO: 视觉联调时需要把 prot_vision.c 计算出的 x/y/z 传进来并更新 arm_ctrl 目标
  */
void arm_vision_callback(void)
{
}
