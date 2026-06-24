/**
  * @file prot_vision.c
  * @version 1.1
  * @date 2025 12 15
	*
  * @brief  视觉信息解算
	*
  *	@author myh
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "prot_vision.h"
#include "crc.h"
#include "control_def.h"
#include "string.h"
#include "math.h"
#include "prot_T_imu.h"
#include "KalmanFilter.h"
#include "math_calcu.h"
#include "mode_switch_task.h"
#include "math.h"
#include "us_tim.h"
#include "prot_dr16.h"
#include "prot_new_rc.h"
#include "prot_rm_judge.h"
#include "prot_dr16.h"
#include "usart.h"
#include "prot_remote_fusion.h"
#include "shoot_task.h"
#include "arm_task.h"
/* Private define ------------------------------------------------------------*/
#ifndef ABS
#define ABS(x)  ((x)>0?(x):-(x))
#endif

#ifndef PI
#define PI      3.14159265358979323846f
#endif
/*  NAN 保护宏  */
#ifndef NAN_PROCESS
#define NAN_PROCESS(now, last)  \
    do { if (isnan(now)) now = last; else last = now; } while (0)
#endif
/* Private variables ---------------------------------------------------------*/
/* 唯一全局变量 */
vision_t vision;
vision_tx_t vision_tx;
vision_mode_e vision_mode;
vision_fruit_msg_t vision_fruit;
/*调试数值*/
uint8_t vis_e[2];
uint32_t cnt_time;
float yaw_diff;
/* Private type --------------------------------------------------------------*/
extern QueueHandle_t VISION_send_queue;
/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
* @brief  视觉数据解包函数  decoding the vision data 
* @param  uint8_t *pVision_Data  串口中断函数传出来的数据缓冲区指针 pointer to the vision_uart_data_buf
* @retval None
*/
void vision_data_handler(uint8_t *pVision_Data,uint32_t len)
{
    static vision_aim_status_e last_aim_status;

	/*新旧视觉切换判断*/
	 if (len >= 2 && pVision_Data[0] == 'S' && pVision_Data[1] == 'P') {
        vision_get_data_sp(pVision_Data, len);
        return;
    }

    /* 采摘水果视觉协议 */
    if (len >= VISION_FRUIT_DATA_LEN && pVision_Data[0] == VISION_SOF) {
        vision_fruit_getData(pVision_Data);
        return;
    }

    if (len < VISION_DATA_LEN) {
        vision.rx_status = TAIL_ERROR;
        return;
    }
    /* 1. 拷帧 */
    memcpy(vision.rx[0].buff, pVision_Data, VISION_DATA_LEN);

    /* 2. 帧尾 & 重复帧检查 */
    if (vision.rx[0].data.eof != 0xCC) {
        vision.rx_status = TAIL_ERROR;
        return;
    }
    if (vision.rx[0].data.cnt == vision.rx[1].data.cnt) {
        if (++vision.rx_repeat_cnt > 10) {
            vision.rx_status = REPEAT_ERROR;
            return;
        }
    }
    vision.online           = 1;
    vision.rx_cnt++;
    vision.rx_status        = NORMAL;
    vision.rx_repeat_cnt    = 0;
    vision.rx[1].data.cnt   = vision.rx[0].data.cnt;
    vision.trace_id         = vision.rx[0].data.trace_id;

    /* 3. NAN 保护 */
    NAN_PROCESS(vision.rx[0].data.yaw,  vision.rx[1].data.yaw);
    NAN_PROCESS(vision.rx[0].data.pit,  vision.rx[1].data.pit);
    NAN_PROCESS(vision.rx[0].data.dis,  vision.rx[1].data.dis);
    NAN_PROCESS(vision.rx[0].data.fire, vision.rx[1].data.fire);
    NAN_PROCESS(vision.rx[0].data.pos,  vision.rx[1].data.pos);

    /* 4. 目标有效 */
    if (vision.rx[0].data.aim_flag) {
        vision.aim_status     = AIMING;
        vision.new_frame_flag = 1;

       /* 4.1 保存视觉原始角度（单位保持视觉帧本身，度） */
        vision.target_yaw_angle = vision.rx[0].data.yaw;
        vision.target_pit_angle = vision.rx[0].data.pit;

 /* 4.2 限幅——默认关闭，用时打开 */
        // if (vsn_drive.target_pit_angle < -23.0f)
        //     vsn_drive.target_pit_angle = -23.0f;
        // if (fabs(vsn_drive.target_yaw_angle - gimbal.pid.yaw_angle_ref * 180.0f / PI) > 99.13f)
        //     vsn_drive.target_yaw_angle = gimbal.pid.yaw_angle_ref * 180.0f / PI;

       /* 4.3 保存视觉下发的误差门限（单位：弧度） */
        vision.min_err = vision.rx[0].data.dis;  // �Ӿ��� dis2 ���ǻ���
//        vision.pit_min_err = vision.rx[0].data.dis2;  // �Ӿ��� dis2 ���ǻ���

        /* 4.4 射击判定：PID 参考 vs 反馈，单位统一为弧度 */
//        float vision_yaw_err = fabsf(gimbal.yaw_angle.ref - gimbal.yaw_angle.fdb);
//        float vision_pit_err = fabsf(gimbal.pit_angle.ref  - gimbal.pit_angle.fdb);

        if (ABS(yaw_err) < vision.min_err && \
            ABS(gimbal.pit_angle.fdb - vision.target_pit_angle) < vision.min_err && \
            vision.rx[0].data.fire != 0)
        {
            vision.shoot_enable = 1;
        } else {
            vision.shoot_enable = 0;
        }
    } else {
        vision.aim_status   = UNAIMING;
        vision.shoot_enable = 0;
    }

    /* 5. 调试标志位（与旧代码一致） */
    vis_e[0] = (ABS(gimbal.yaw_angle.fdb- vision.target_yaw_angle * PI / 180.0f) < vision.min_err) ? 1 : 0;
    vis_e[1] = (ABS(gimbal.pit_angle.fdb  - vision.target_pit_angle * PI / 180.0f) < vision.min_err) ? 1 : 0;

    /* 6. 首次丢失 */
    if (last_aim_status == AIMING && vision.aim_status == UNAIMING)
        vision.aim_status = FIRST_LOST;

    last_aim_status = vision.aim_status;
}

/**
* @brief  云台任务状态机用的视觉控制状态机  aimming mode calculation function
* @param  None
* @retval None
*/
void vsn_gimbal_ref_calc(void)
{
    switch (vision.aim_status){
    case AIMING:
        if (vision.new_frame_flag){
		yaw_diff = fabsf(circle_error(vision.target_yaw_angle, gimbal.yaw_angle.fdb, 360));
        if (yaw_diff > 90.0f) {
            vision.new_frame_flag = 0;
			gimbal.pit_angle.ref = gimbal.pit_angle.fdb;  // 当前周期立刻停住
            gimbal.yaw_angle.ref = gimbal.yaw_angle.fdb;
			gimbal.yaw_vel_feedforward = 0.0f;
            break;
        }
            gimbal.pit_angle.ref = vision.target_pit_angle;//* PI / 180.0f
            gimbal.yaw_angle.ref = vision.target_yaw_angle; // * PI / 180.0f
			gimbal.yaw_vel_feedforward = vision.rx[0].data.yaw_vel;
            vision.new_frame_flag = 0;
        }
        break;

    case FIRST_LOST:
        vision.aim_status = UNAIMING;
        gimbal.pit_angle.ref = gimbal.pit_angle.fdb;
        gimbal.yaw_angle.ref = gimbal.yaw_angle.fdb;
		gimbal.yaw_vel_feedforward = 0.0f;
        break;

    case UNAIMING:
       /* 丢失目标：遥控增量 */
        if (remote_fusion.sw.sw1 == RC_DN){
            gimbal.yaw_angle.ref += (-remote_fusion.mouse.x) * KEYBOARD_SCALE_YAW;
            gimbal.pit_angle.ref += (-remote_fusion.mouse.y) * KEYBOARD_SCALE_PIT;
        }else{
            gimbal.yaw_angle.ref += remote_fusion.channel.ch1 * scale.ch1 * (-1);
            gimbal.pit_angle.ref += remote_fusion.channel.ch2  * scale.ch2 * (-1);
        }
		gimbal.yaw_vel_feedforward = 0.0f;
        break;
    }
}


/**
* @brief  选择优先级更高的目标  Choose the target with higher priority
* @param  None
* @retval None
*/
void Trace_ID_Select_OLD(RemoteFusion_Handle_t *remote_fusion)
{
	if(remote_fusion->keyboard.bit.Z)  vision_tx.data.trace_id = 7;  // W - 前哨站
    if(remote_fusion->keyboard.bit.X)  vision_tx.data.trace_id = 2;  // Z - 前哨站前装甲
    if(remote_fusion->keyboard.bit.C)  vision_tx.data.trace_id = 3;  // A - 步兵 3号
    if(remote_fusion->keyboard.bit.F)  vision_tx.data.trace_id = 1;  // D - 右侧，大符位，英雄
//    if(remote_fusion->keyboard.bit.S)  vision_tx.data.trace_id = 2;  // S - 哨兵站，己方前哨站的攻击
}

/**
* @brief  视觉数据打包  Vision data packaging
* @param  None
* @retval None
*/
void vision_output_data(void)
{
    /* 1. 帧头/尾固定 */
    vision_tx.data.SOF  = 0x11;
    vision_tx.data.eof1 = 0x22;
    vision_tx.data.eof2 = 0x33;

    /* 2. 浮点→小端 4 字节（与旧代码符号一致） */
    float tmp = -imu_data.pitch;           memcpy(vision_tx.data.imu_pit,     &tmp, 4);
    tmp = -imu_data.yaw;                   memcpy(vision_tx.data.imu_yaw,     &tmp, 4);
    tmp = -imu_data.wy;                    memcpy(vision_tx.data.imu_pit_spd, &tmp, 4);
    tmp = -imu_data.wz;                    memcpy(vision_tx.data.imu_yaw_spd, &tmp, 4);
    tmp = Shoot_Data.bullet_speed;         memcpy(vision_tx.data.shoot_speed, &tmp, 4);

    /* 3. 位域+单字节 */
	if(robot_status.robot_id>100)
	{
		vision_tx.data.camp = 1;
	}
	else{
		vision_tx.data.camp = 0;
	}
    vision_tx.data.aiming_mode   = vision_mode;             // 0:自瞄 1:大能量 2:小能量 4:反陀螺  这是抄步兵的，飞机记得后面改一下
    vision_tx.data.shooter_speed = 3;                       // 固定 3 档
    vision_tx.data.vacancy       = 0;                       // 预留，填 0
    

	Trace_ID_Select_OLD(&remote_fusion);
//    vision_tx.data.trace_id      = vision.trace_id;         // 来自视觉包

    /* 4. 串口DMA 发出 (视觉改接U6) */
    HAL_UART_Transmit_DMA(&huart6, vision_tx.buff, 25);
	vision.tx_cnt++;
//    /* 4. 使用usb 发出 */
//    if (xQueueSend(VISION_send_queue,vision_tx.buff, 10) == pdTRUE) 
//    {
//        vision.tx_cnt++;
//    }
//    else
//    {
//            // 队列满，错误处理：丢弃这帧新数据，继续下一帧
//    // 可选：统计丢包次数，用于调试
//    // static uint32_t drop_count = 0;
//    // drop_count++;
//    }

	cnt_time=osKernelSysTick()/1000;//单位：秒
    vision.tx_freq=vision.tx_cnt/cnt_time;
	vision.rx_freq=vision.rx_cnt/cnt_time;
}

/*---------新视觉代码--------------------------*/

vision_rx_msg_sp_t vision_rx_msg_sp;
vision_tx_msg_sp_t vision_tx_msg_sp;
VISION_SP_RX_MODE_e VISION_SP_RX_MODE;
//static uint8_t vision_get_sp_mode(void)
//{
//    if (ctrl_mode == KEYBOARD_MODE) {
//        if (KEY_PRESS_VISION1) {
//            return vMODE_bENERGY;
//        } else if (KEY_PRESS_VISION2) {
//            return vMODE_sENERGY;
//        } else if (rc.mouse.r == 1) {
//            return vMODE_AUTO;
//        }
//    } else if (ctrl_mode == REMOTER_MODE) {
//        if (rc_fsm_check(RC_LEFT_LD) && rc_fsm_check(RC_RIGHT_RD)) {
//            return vMODE_AUTO;
//        }
//    }

//    return vMODE_ANTISPIN;
//}

static void vision_euler_to_quat_wxyz(float yaw, float pitch, float roll, float q[4])
{
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);

    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}
static float vision_normalize_yaw(float yaw)
{
    while (yaw > 2.0f * PI) {
        yaw -= 2.0f * PI;
    }
    while (yaw < 0.0f) {
        yaw += 2.0f * PI;
    }
    return yaw;
}

static uint8_t vision_sp_control_enabled(uint8_t mode)
{
    return (mode == VISION_SP_RX_MODE_CONTROL || mode == VISION_SP_RX_MODE_FIRE);
}
static uint8_t vision_sp_float_valid(float value)
{
    return (value > -10000.0f && value < 10000.0f);
}

static float vision_sp_read_float(const uint8_t data[4])
{
    float value;
    memcpy(&value, data, sizeof(value));
    return value;
}

static void vision_sp_write_float(uint8_t data[4], float value)
{
    memcpy(data, &value, sizeof(value));
}

static void vision_sp_write_u16(uint8_t data[2], uint16_t value)
{
    data[0] = (uint8_t)(value & 0xffU);
    data[1] = (uint8_t)((value >> 8) & 0xffU);
}

static void vision_get_data_sp(uint8_t *data, uint32_t len)
{
    vision.last_rx_tick = HAL_GetTick();
    static vision_aim_status_e last_aim_status;

    if (len < sizeof(vision_rx_msg_sp_t)) {
        vision.rx_status = TAIL_ERROR;
        return;
    }

    if (!Verify_CRC16_Check_Sum(data, sizeof(vision_rx_msg_sp_t))) {
        vision.rx_status = TAIL_ERROR;
        return;
    }

    memcpy(&vision_rx_msg_sp, data, sizeof(vision_rx_msg_sp));

    float rx_yaw = vision_sp_read_float(vision_rx_msg_sp.yaw);
    float rx_yaw_vel = vision_sp_read_float(vision_rx_msg_sp.yaw_vel);
    float rx_yaw_acc = vision_sp_read_float(vision_rx_msg_sp.yaw_acc);
    float rx_pitch = vision_sp_read_float(vision_rx_msg_sp.pitch);
    float rx_pitch_vel = vision_sp_read_float(vision_rx_msg_sp.pitch_vel);
    float rx_pitch_acc = vision_sp_read_float(vision_rx_msg_sp.pitch_acc);

    if (vision_rx_msg_sp.mode > VISION_SP_RX_MODE_FIRE ||
        !vision_sp_float_valid(rx_yaw) ||
        !vision_sp_float_valid(rx_yaw_vel) ||
        !vision_sp_float_valid(rx_yaw_acc) ||
        !vision_sp_float_valid(rx_pitch) ||
        !vision_sp_float_valid(rx_pitch_vel) ||
        !vision_sp_float_valid(rx_pitch_acc)) {
        vision.rx_status = TAIL_ERROR;
        return;
    }

    vision.online = 1;
    vision.rx_status = NORMAL;
    vision.rx_repeat_cnt = 0;
    vision.trace_id = 0;

    if (vision_sp_control_enabled(vision_rx_msg_sp.mode)) {
        vision.aim_status = AIMING;
        vision.new_frame_flag = 1;
        vision.target_yaw_angle = vision_normalize_yaw(rx_yaw) / PI * 180.0f;
        vision.target_pit_angle = rx_pitch / PI * 180.0f;

        if (vision.target_pit_angle < -15.0f) {
            vision.target_pit_angle = -15.0f;
        }
        if (fabs(circle_error(vision.target_yaw_angle, gimbal.yaw_angle.fdb, 360)) > 99.0f) {
            vision.target_yaw_angle = gimbal.yaw_angle.fdb;
        }

        vision.rx[0].data.yaw = rx_yaw / PI * 180.0f;
        vision.rx[0].data.pit = rx_pitch / PI * 180.0f;
        vision.rx[0].data.yaw_vel = rx_yaw_vel / PI * 180.0f;
        vision.rx[0].data.yaw_acc = rx_yaw_acc;
        vision.rx[0].data.pit_vel = rx_pitch_vel / PI * 180.0f;
        vision.rx[0].data.pit_acc = rx_pitch_acc;
        vision.yaw_min_err = VISION_SP_FIRE_YAW_ERR;
        vision.pit_min_err = VISION_SP_FIRE_PIT_ERR;

        if (ABS(circle_error(vision.target_yaw_angle, gimbal.yaw_angle.fdb, 360)) < vision.yaw_min_err) {
            vis_e[0] = 1;
        } else {
            vis_e[0] = 0;
        }
        if (ABS(gimbal.pit_angle.fdb - vision.target_pit_angle) < vision.pit_min_err) {
            vis_e[1] = 1;
        } else {
            vis_e[1] = 0;
        }

        if (vision_rx_msg_sp.mode == VISION_SP_RX_MODE_FIRE && vis_e[0] && vis_e[1]) {
            vision.shoot_enable = 1;
        } else {
            vision.shoot_enable = 0;
        }
    } else {
        vision.aim_status = UNAIMING;
        vision.new_frame_flag = 0;
        vision.shoot_enable = 0;
        vision.rx[0].data.yaw_vel = 0.0f;
        vision.rx[0].data.yaw_acc = 0.0f;
        vision.rx[0].data.pit_vel = 0.0f;
        vision.rx[0].data.pit_acc = 0.0f;
        vis_e[0] = 0;
        vis_e[1] = 0;
    }

    if (vision_rx_msg_sp.mode == VISION_SP_RX_MODE_FIRE) {
//        fire_Cnt++;
    }
    if (last_aim_status == AIMING && vision.aim_status == UNAIMING) {
        vision.aim_status = FIRST_LOST;
    }
    last_aim_status = vision.aim_status;
    vision.last_rx_tick = HAL_GetTick();
}
float yaw_tx;
void vision_output_data_sp(void)
{
    float q[4];
#ifdef VISION_HAND_EYE_CALIBRATION
    yaw_tx = (float)(8192 - yaw_motor.ecd) / 8192.0f * 2.0f * PI;
#else
    yaw_tx = imu_data.yaw;
#endif

    vision_tx_msg_sp.head[0] = 'S';
    vision_tx_msg_sp.head[1] = 'P';
//    vision_tx_msg_sp.mode = vision_get_sp_mode();
	vision_tx_msg_sp.mode = VISION_SP_MODE_AUTO_AIM;
	vision_euler_to_quat_wxyz(yaw_tx * PI / 180.0f, imu_data.pitch * PI / 180.0f, imu_data.roll * PI / 180.0f, q);
    for (uint8_t i = 0; i < 4; i++) {
    vision_sp_write_float(vision_tx_msg_sp.q[i], q[i]);
    }

    vision_sp_write_float(vision_tx_msg_sp.yaw, yaw_tx* PI / 180.0f);
    vision_sp_write_float(vision_tx_msg_sp.yaw_vel, imu_data.wz* PI / 180.0f);
    vision_sp_write_float(vision_tx_msg_sp.pitch, imu_data.pitch* PI / 180.0f);
    vision_sp_write_float(vision_tx_msg_sp.pitch_vel, imu_data.wy* PI / 180.0f);
    vision_sp_write_float(vision_tx_msg_sp.bullet_speed, Shoot_Data.bullet_speed);
    vision_sp_write_u16(vision_tx_msg_sp.bullet_count, (uint16_t)shoot.barrel.heat_remain);
    if (robot_status.robot_id > 100) {
        vision_tx_msg_sp.camp = 1;
    } else {
        vision_tx_msg_sp.camp = 0;
    }
     Append_CRC16_Check_Sum((uint8_t *)&vision_tx_msg_sp, sizeof(vision_tx_msg_sp));

    HAL_UART_Transmit_DMA(&huart6, (uint8_t *)&vision_tx_msg_sp, VISION_SP_TX_DATA_LEN);  /* 视觉改接U6 */
	
//	if (xQueueSend(VISION_send_queue, &vision_tx_msg_sp, 10) == pdTRUE) {
//    vision.tx_cnt++;
//	}
}

/**
* @brief  采摘水果视觉数据解析
* @param  buff: 接收到的数据缓冲区
* @retval None
*/
void vision_fruit_getData(uint8_t *buff)
{
    if(buff[0] == VISION_SOF && buff[VISION_FRUIT_DATA_LEN-1] == VISION_EOF)
    {
        vision_fruit.flag = buff[5];
        if(vision_fruit.flag)
        {
            vision_fruit.x_dis = (int16_t)((buff[1] << 8) | buff[2]);
            vision_fruit.y_dis = (int16_t)((buff[3] << 8) | buff[4]);
            vision_fruit.rec_cnt++;
            
            /* ============================================
             * 临时视觉限幅 (相机像素坐标差)
             * 相机分辨率: 320x240, 输出的是 Δx, Δy
             * 此限幅值为临时值，后续与视觉联调时再调整
             * ============================================ */
            float x = (float)vision_fruit.x_dis;
            float y = (float)vision_fruit.y_dis;
            
            /* 限幅: ±160 (X), ±120 (Y) */
            if (x > 160.0f)  x = 160.0f;
            if (x < -160.0f) x = -160.0f;
            if (y > 120.0f)  y = 120.0f;
            if (y < -120.0f) y = -120.0f;
            
            /* TODO: 添加工作空间检查，确保目标位置在机械臂可达范围内 */
            /* 工作空间: L1+L2=300mm，需要检查 sqrt(x²+y²) <= 300 */
            float distance = sqrtf(x*x + y*y);
            if (distance > 300.0f) {
                /* 超出工作空间，按比例缩放到边界 */
                float scale = 300.0f / distance;
                x *= scale;
                y *= scale;
            }
            
            float z = 0.0f;  /* Z轴默认值，后续可由视觉提供 */
            
            /* 调用机械臂视觉回调函数 */
            extern void arm_vision_callback(void);
            arm_vision_callback();
        }
        else
            return;
    }
}

