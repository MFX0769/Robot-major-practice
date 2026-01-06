/**
  * @file prot_vision.c
  * @version 1.1
  * @date 2025 12 15
	*
  * @brief  �Ӿ���Ϣ����
	*
  *	@author
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "prot_vision.h"
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
/* Private define ------------------------------------------------------------*/
#ifndef ABS
#define ABS(x)  ((x)>0?(x):-(x))
#endif

#ifndef PI
#define PI      3.14159265358979323846f
#endif
/*  NAN  */
#ifndef NAN_PROCESS
#define NAN_PROCESS(now, last)  \
    do { if (isnan(now)) now = last; else last = now; } while (0)
#endif
/* Private variables ---------------------------------------------------------*/
/* Ψһȫ�ֱ��� */
vision_t vision;
vision_tx_t vision_tx;
vision_mode_e vision_mode;
/*������ֵ*/
uint8_t vis_e[2];
uint32_t cnt_time;
/* Private type --------------------------------------------------------------*/
extern QueueHandle_t VISION_send_queue;
/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
* @brief  �Ӿ����ݽ������  decoding the vision data 
* @param  uint8_t *pVision_Data  �����жϺ��������������ݻ�����ָ�� pointer to the vision_uart_data_buf
* @retval None
*/
void vision_data_handler(uint8_t *pVision_Data)
{
    static vision_aim_status_e last_aim_status;

    /* 1. ��֡ */
    memcpy(vision.rx[0].buff, pVision_Data, VISION_DATA_LEN);

    /* 2. ֡β & �ظ�֡��� */
    if (vision.rx[0].data.eof != 0xCC) {
        vision.rx_status = TAIL_ERROR;
        return;
    }
    if (vision.rx[0].data.cnt == vision.rx[1].data.cnt) {
        if (++vision.rx_cnt > 10) {
            vision.rx_status = REPEAT_ERROR;
            return;
        }
    }
    vision.online           = 1;
    vision.rx_cnt++;
    vision.rx_status        = NORMAL;
    vision.rx_cnt    = 0;
    vision.rx[1].data.cnt   = vision.rx[0].data.cnt;
    vision.trace_id         = vision.rx[0].data.trace_id;

    /* 3. NAN ���� */
    NAN_PROCESS(vision.rx[0].data.yaw,  vision.rx[1].data.yaw);
    NAN_PROCESS(vision.rx[0].data.pit,  vision.rx[1].data.pit);
    NAN_PROCESS(vision.rx[0].data.dis,  vision.rx[1].data.dis);
    NAN_PROCESS(vision.rx[0].data.fire, vision.rx[1].data.fire);
    NAN_PROCESS(vision.rx[0].data.pos,  vision.rx[1].data.pos);

    /* 4. Ŀ����Ч */
    if (vision.rx[0].data.aim_flag) {
        vision.aim_status     = AIMING;
        vision.new_frame_flag = 1;

        /* 4.1 �����Ӿ�ԭʼ�Ƕȣ���λ�����Ӿ�֡�������ȣ� */
        vision.target_yaw_angle = vision.rx[0].data.yaw;
        vision.target_pit_angle = vision.rx[0].data.pit;

        /* 4.2 �޷�����Ĭ�Ϲرգ���ʱ�� */
        // if (vsn_drive.target_pit_angle < -23.0f)
        //     vsn_drive.target_pit_angle = -23.0f;
        // if (fabs(vsn_drive.target_yaw_angle - gimbal.pid.yaw_angle_ref * 180.0f / PI) > 99.13f)
        //     vsn_drive.target_yaw_angle = gimbal.pid.yaw_angle_ref * 180.0f / PI;

        /* 4.3 �����Ӿ��·���������ޣ���λ�����ȣ� */
        vision.yaw_min_err = vision.rx[0].data.dis;   // �Ӿ��� dis ���ǻ���
        vision.pit_min_err = vision.rx[0].data.dis2;  // �Ӿ��� dis2 ���ǻ���

        /* 4.4 ����ж���PID �ο� vs ��������λͳһΪ���� */
        float yaw_err = fabsf(gimbal.yaw_angle.ref - gimbal.yaw_angle.fdb);
        float pit_err = fabsf(gimbal.pit_angle.ref  - gimbal.pit_angle.fdb);

        if (yaw_err < vision.yaw_min_err &&
            pit_err < vision.pit_min_err &&
            (vision.rx[0].data.fire == 1 || vision.rx[0].data.fire == 2))
        {
            vision.shoot_enable = 1;
        } else {
            vision.shoot_enable = 0;
        }
    } else {
        vision.aim_status   = UNAIMING;
        vision.shoot_enable = 0;
    }

    /* 5. ���Ա�־λ����ɴ���һ�£� */
    vis_e[0] = (ABS(gimbal.yaw_angle.fdb- vision.target_yaw_angle * PI / 180.0f) < vision.yaw_min_err) ? 1 : 0;
    vis_e[1] = (ABS(gimbal.pit_angle.fdb  - vision.target_pit_angle * PI / 180.0f) < vision.pit_min_err) ? 1 : 0;

    /* 6. �״ζ�ʧ */
    if (last_aim_status == AIMING && vision.aim_status == UNAIMING)
        vision.aim_status = FIRST_LOST;

    last_aim_status = vision.aim_status;
}

/**
* @brief  ��̨����״̬���õ��Ӿ�����״̬��  aimming mode calculation function
* @param  None
* @retval None
*/
void vsn_gimbal_ref_calc(void)
{
    switch (vision.aim_status){
    case AIMING:
        if (vision.new_frame_flag){
            gimbal.pit_angle.ref = vision.target_pit_angle;//* PI / 180.0f
            gimbal.yaw_angle.ref = vision.target_yaw_angle; // * PI / 180.0f
            vision.new_frame_flag = 0;
        }
        break;

    case FIRST_LOST:
        vision.aim_status = UNAIMING;
        gimbal.pit_angle.ref = gimbal.pit_angle.fdb;
        gimbal.yaw_angle.ref = gimbal.yaw_angle.fdb;
        break;

    case UNAIMING:
        /* ��ʧĿ�꣺ң������ */
        if (rc.sw1 == RC_DN){
            gimbal.yaw_angle.ref += (-rc.mouse.x) * KEYBOARD_SCALE_YAW;
            gimbal.pit_angle.ref += (-rc.mouse.y) * KEYBOARD_SCALE_PIT;
        }else{
            gimbal.yaw_angle.ref += rc.ch1 * scale.ch1 * (-1);
            gimbal.pit_angle.ref += rc.ch2 * scale.ch2 * (-1);
        }
        break;
    }
}

//void Trace_ID_Select_NEW(new_rc_info_t *new_rc)
//{
//	if(new_rc->key_W)  vision_tx.data.trace_id  = 7;
//	if(new_rc->key_A)  vision_tx.data.trace_id  = 3;
//	if(new_rc->key_Z)  vision_tx.data.trace_id  = 4;
//	if(new_rc->key_D)  vision_tx.data.trace_id  = 1;
//	if(new_rc->key_S)  vision_tx.data.trace_id  = 2;	
//}


/**
* @brief  ѡ�����ȼ����ߵ�Ŀ��  Choose the target with higher priority
* @param  None
* @retval None
*/
void Trace_ID_Select_OLD(rc_info_t *rc)
{
	if(rc->kb.bit.W)  vision_tx.data.trace_id  = 7;         //"w"��ζ��ǰ�棺ǰ��վ
	if(rc->kb.bit.Z)  vision_tx.data.trace_id  = 4;         //"z"����ζ�����¹����Ĳ��� 4��
	if(rc->kb.bit.A)  vision_tx.data.trace_id  = 3;         //"a"��ζ����ߣ����¹����Ĳ��� 3��
	if(rc->kb.bit.D)  vision_tx.data.trace_id  = 1;         //"d"��ζ���ұߣ����ڵ���λ��Ӣ��
	if(rc->kb.bit.S)  vision_tx.data.trace_id  = 2;	        //"s"��ζ�ź��棺�������Ķһ�վ�Ĺ���
}

/**
* @brief  �Ӿ����ݴ��  Vision data packaging
* @param  None
* @retval None
*/
void vision_output_data(void)
{
    /* 1. ֡ͷ/β�̶� */
    vision_tx.data.SOF  = 0x11;
    vision_tx.data.eof1 = 0x22;
    vision_tx.data.eof2 = 0x33;

    /* 2. �����С�� 4 �ֽڣ���ɴ������һ�£� */
    float tmp = -imu_data.pitch;           memcpy(vision_tx.data.imu_pit,     &tmp, 4);
    tmp = -imu_data.yaw;                   memcpy(vision_tx.data.imu_yaw,     &tmp, 4);
    tmp = -imu_data.wy;                    memcpy(vision_tx.data.imu_pit_spd, &tmp, 4);
    tmp = -imu_data.wz;                    memcpy(vision_tx.data.imu_yaw_spd, &tmp, 4);
    tmp = Shoot_Data.bullet_speed;         memcpy(vision_tx.data.shoot_speed, &tmp, 4);

    /* 3. λ��+���ֽ� */
    vision_tx.data.camp = 0;
    vision_tx.data.aiming_mode   = vision_mode;             // 0:���� 1:������ 2:С���� 4:������  ���ǳ������ģ��ɻ��ǵú����һ��
    vision_tx.data.shooter_speed = 3;                       // �̶� 3 ��
//    vision_tx.data.vacancy       = 0;                       // Ԥ������ 0
    

	Trace_ID_Select_OLD(&rc);
//    vision_tx.data.trace_id      = vision.trace_id;         // �����Ӿ���

    /* 4. ����DMA ���� */
//  HAL_UART_Transmit_DMA(&huart10, vision_tx.buff, 25);

    /* 4. ʹ��usb ���� */
    if (xQueueSend(VISION_send_queue,vision_tx.buff, 10) == pdTRUE) 
    {
        vision.tx_cnt++;
    }
    else
    {
            // ����������������������֡�����ݣ�������һ֡
    // ��ѡ��ͳ�ƶ������������ڵ���
    // static uint32_t drop_count = 0;
    // drop_count++;
    }


	
	cnt_time=osKernelSysTick()/1000;//��λ����
    vision.tx_freq=vision.tx_cnt/cnt_time;
	vision.rx_freq=vision.rx_cnt/cnt_time;

}
