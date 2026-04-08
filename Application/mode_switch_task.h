#ifndef __MODESWITCH_TASK_H__
#define __MODESWITCH_TASK_H__

#include "stdint.h"

typedef enum
{
    PROTECT_MODE,   //保护模式
    REMOTER_MODE,   //遥控模式
    KEYBOARD_MODE,  //键盘模式
    VISION_MODE     //视觉模式(鼠标右键开启)
} ctrl_mode_e;

typedef enum
{
	DIS_NEAR,
	DIS_FAR
}_gimbal_ctrl_;

void mode_switch_task(void const *argu);
void fly_led_status(void);

void vtm_mode_handler(void); //由拨杆1决定系统模式切换，主要是云台、底盘和发射
void vtm_unlock_init(void);
void Rc_Fail_mode_handler(void);  //由拨杆1决定系统模式切换，主要是云台、底盘和发射器
void vtm_safe_lock(void);
extern uint8_t pit_init_flag; 
extern uint8_t lock_flag;
extern ctrl_mode_e ctrl_mode;
extern _gimbal_ctrl_ gimbal_ctrl_mode;
extern uint8_t RC_FAIL_KEYBOARD_FLAG;

#endif
