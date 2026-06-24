#ifndef _CONTROL_DEF_H_
#define _CONTROL_DEF_H_

#include "stm32h7xx.h"
#include "math_calcu.h"



/*----------------------------- the whole system ----------------------------- */
// task period
#define GIMBAL_PERIOD	    1
#define CHASSIS_PERIOD    2
#define SHOOT_PERIOD      1
#define USART_SEND_PERIOD 2
#define MODESWITCH_PERIOD 6
#define STATUS_PERIOD     500



/*----------------------------- manipulator preference ----------------------------- */
#define ARM_TASK_PERIOD    3       /* 机械臂任务周期 (ms) */
#define ARM_ENABLE         1       /* 机械臂使能: 1=启用, 0=禁用 */

/* special function key and key status definition */
// chassis control key (status) define
#define KEY_CHASSIS_FIGHT       KB_C  //�����ֲ���
#define FLAG_CHASSIS_FIGHT      kb_status[KEY_CHASSIS_FIGHT]

#define KEY_CHASSIS_ROTATE      KB_F
#define FLAG_CHASSIS_ROTATE     kb_status[KEY_CHASSIS_ROTATE]

// shoot control key (status) defineb
#define KEY_SHOOT_FRIC          KB_V
#define FLAG_SHOOT_FRIC         kb_status[KEY_SHOOT_FRIC]

#define KEY_SHOOT_HOUSE         KB_B
#define FLAG_SHOOT_HOUSE        kb_status[KEY_SHOOT_HOUSE]

// vision control key status define
#define KEY_VISION_sENERGY      KB_Q
#define FLAG_VISION_sENERGY     kb_status[KEY_VISION_sENERGY]

#define KEY_VISION_bENERGY      KB_E
#define FLAG_VISION_bENERGY     kb_status[KEY_VISION_bENERGY]

#define KEY_VISION_ENERGY_DIR   KB_Z

#define KEY_VISION_ANTIROTATE   KB_R
#define FLAG_VISION_ANTIROTATE  kb_status[KEY_VISION_ANTIROTATE]

#define KEY_VISION_SENTRY       KB_X
#define FLAG_VISION_SENTRY      kb_status[KEY_VISION_SENTRY]


#define HIGH_SPEED	 5900 // 2025��5��15�� ����ʵ������21~23 5900





///* ����Ƶ�� */
#define TRIGGER_PERIOD   70  //TRIGGER_PERIODԽ����ƵԽ�����ݲ�֧��ֱ�Ӽ��㣬��Ҫ��ǹ�ܲ���
								//80��25�������Ƶ
#define TRIGGER_SPEED    5//2025��4��10�� �ڴ�����֮�󣬲������ٶȲ�����TRIGGER_SPEED������������TRIGGER_PERIOD����
//����ʱ�ɲ����ٶ�6800��˫��7600   std��7300   //2025��1��20�� 7500
/*-----------------------------chassis---------------------------*/
#define RC_CH4_SCALE    12
#define RC_CH3_SCALE    12

/* SBUS麦轮底盘速度映射：摇杆满偏(~670)*SCALE=target_rpm，要跟实测电流-转速量级匹配(几千rpm)，不能太小 */
#define CHASSIS_SCALE_VX   3.0f
#define CHASSIS_SCALE_VY   3.0f
#define CHASSIS_SCALE_WZ   4.0f
#define MECANUM_L          1.0f

/*-----------------------------gimbal----------------------------*/
//IMU_MODE_SCALE
#define RC_CH1_SCALE_IMU                0.0002f    //��̨����Ƶ��2msʱ 0.0002f
#define RC_CH2_SCALE_IMU                0.0002f 	// 0.00001f  �����Ϊ�Ƕȷ���������ģʽ	��̨����Ƶ��2msʱ 0.0002f
#define RC_CH3_SCALE_IMU                0.00000000000000000000000001f // 

//ECD_MODE_SCALE
#define RC_CH1_SCALE_ECD                0.000000000000000000000004f
#define RC_CH2_SCALE_ECD                0.000000000000000005f    


//KEYBOARD_MODE_SCALE
#define KEYBOARD_SCALE_PIT          ( 0.0003f )    //��̨����Ƶ��2msʱ0.0004f
#define KEYBOARD_SCALE_YAW	        ( 0.0003f )		//��̨����Ƶ��2msʱ0.0004f
#define KEYBOARD_SCALE_YAW_SUPPLY   ( -0.0003f )

#define GIMBAL_PIT_CENTER_OFFSET    3400         //���������ֵ���޷�,���ʵ��Сһ��㣬��ֹ����ѻ�е����
#define GIMBAL_PIT_MAX              1.7f
#define GIMBAL_PIT_MIN              2.5f

//#define GIMBAL_YAW_MAX              6800
//#define GIMBAL_YAW_MIN              2750
#define GIMBAL_YAW_CENTER_OFFSET    6000    
#define GIMBAL_YAW_BETWEEN_ECD      ( 8191 / 8 )
#define FIGHT_OFFSET_ERR            ( -1.0f * GIMBAL_YAW_BETWEEN_ECD / 8191 * 2 * PI )\


#define IMU_feedback 1  //2025����û��ά��������feedback�Ĵ��룬����ֱ��ʹ�ã�IMU_feedbackӦ�ñ���Ϊ1 

#endif


