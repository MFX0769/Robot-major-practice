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
/* special function key and key status definition */
// chassis control key (status) define
#define KEY_CHASSIS_FIGHT       KB_C  //操作手不用
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


#define HIGH_SPEED	 6000 // 2025年5月15日 比赛实际射速21~23





///* 拨盘频率 */
#define TRIGGER_PERIOD   150  //TRIGGER_PERIOD越大，射频越慢，暂不支持直接计算，需要靠枪管测算
								//60，25年国赛射频
#define TRIGGER_SPEED    5//2025年4月10日 在此日期之后，拨弹盘速度不再由TRIGGER_SPEED决定，而是由TRIGGER_PERIOD决定
//连发时旧拨盘速度6800，双层7600   std：7300   //2025年1月20日 7500
/*-----------------------------chassis---------------------------*/
#define RC_CH4_SCALE    12     
#define RC_CH3_SCALE    12

/*-----------------------------gimbal----------------------------*/
//IMU_MODE_SCALE
#define RC_CH1_SCALE_IMU                0.0002f    //云台任务频率2ms时 0.0002f
#define RC_CH2_SCALE_IMU                0.0002f 	// 0.00001f  电机作为角度反馈、力矩模式	云台任务频率2ms时 0.0002f
#define RC_CH3_SCALE_IMU                0.00000000000000000000000001f // 

//ECD_MODE_SCALE
#define RC_CH1_SCALE_ECD                0.000000000000000000000004f
#define RC_CH2_SCALE_ECD                0.000000000000000005f    


//KEYBOARD_MODE_SCALE
#define KEYBOARD_SCALE_PIT          ( 0.0002f )    //云台任务频率2ms时0.0004f
#define KEYBOARD_SCALE_YAW	        ( 0.0002f )		//云台任务频率2ms时0.0004f
#define KEYBOARD_SCALE_YAW_SUPPLY   ( -0.0003f )

#define GIMBAL_PIT_CENTER_OFFSET    3400         //电机编码器值的限幅,会比实际小一点点，防止代码把机械带走
#define GIMBAL_PIT_MAX              1.7f
#define GIMBAL_PIT_MIN              2.5f

//#define GIMBAL_YAW_MAX              6800
//#define GIMBAL_YAW_MIN              2750
#define GIMBAL_YAW_CENTER_OFFSET    6000    
#define GIMBAL_YAW_BETWEEN_ECD      ( 8191 / 8 )
#define FIGHT_OFFSET_ERR            ( -1.0f * GIMBAL_YAW_BETWEEN_ECD / 8191 * 2 * PI )\


#define IMU_feedback 1  //2025赛季没有维护编码器feedback的代码，请勿直接使用，IMU_feedback应该保持为1 

#endif


