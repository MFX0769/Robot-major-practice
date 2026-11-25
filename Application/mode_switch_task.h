#ifndef __MODESWITCH_TASK_H__
#define __MODESWITCH_TASK_H__

#include <stdint.h>

typedef enum
{
    PROTECT_MODE,    /* 保护: 断舵机使能, 云台/底盘零输出 */
    REMOTER_MODE,    /* 左拨杆上: 遥控 */
    KEYBOARD_MODE,   /* 保留兼容, 当前无键盘 */
    VISION_MODE,     /* 保留视觉接口 */
    ARM_TEACH_MODE   /* 左拨杆下: 肩肘释力拖动示教 */
} ctrl_mode_e;

typedef enum
{
    DIS_NEAR,
    DIS_FAR
} _gimbal_ctrl_;

void mode_switch_task(void const *argu);
void fly_led_status(void);

/* 保留旧任务接口, DBUS 模式下不走图传遥控分支。 */
void vtm_mode_handler(void);
void vtm_unlock_init(void);
void Rc_Fail_mode_handler(void);
void vtm_safe_lock(void);

extern uint8_t pit_init_flag;
extern uint8_t lock_flag;
extern ctrl_mode_e ctrl_mode;
extern uint8_t gimbal_ctrl_mode;
extern uint8_t RC_FAIL_KEYBOARD_FLAG;

/* 无遥控器时的显式模式覆盖（示教用），clear 后回 PROTECT */
void mode_user_set(ctrl_mode_e mode);
void mode_user_clear(void);
uint8_t mode_user_active(void);

#endif
