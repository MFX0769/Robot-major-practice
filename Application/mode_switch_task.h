#ifndef __MODESWITCH_TASK_H__
#define __MODESWITCH_TASK_H__

#include "stdint.h"

typedef enum
{
    PROTECT_MODE,   //����ģʽ
    REMOTER_MODE,   //ң��ģʽ
    KEYBOARD_MODE,  //����ģʽ
    VISION_MODE     //�Ӿ�ģʽ(����Ҽ�����)
} ctrl_mode_e;

typedef enum
{
	DIS_NEAR,
	DIS_FAR
}_gimbal_ctrl_;

void mode_switch_task(void const *argu);
void fly_led_status(void);

void vtm_mode_handler(void); //�ɲ���1����ϵͳģʽ�л�����Ҫ����̨�����̺ͷ���
void vtm_unlock_init(void);
void Rc_Fail_mode_handler(void);  //�ɲ���1����ϵͳģʽ�л�����Ҫ����̨�����̺ͷ�����
void vtm_safe_lock(void);
extern uint8_t pit_init_flag; 
extern uint8_t lock_flag;
extern ctrl_mode_e ctrl_mode;
extern _gimbal_ctrl_ gimbal_ctrl_mode;
extern uint8_t RC_FAIL_KEYBOARD_FLAG;

#endif
