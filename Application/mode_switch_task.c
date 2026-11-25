#include "mode_switch_task.h"
#include "prot_dbus.h"
#include "prot_dr16.h"
#include "prot_sbus.h"
#include "prot_vision.h"
#include "prot_ws2812.h"
#include "status_task.h"
#include "cmsis_os.h"

ctrl_mode_e ctrl_mode;
uint8_t lock_flag;
uint8_t RC_FAIL_KEYBOARD_FLAG;
uint8_t gimbal_ctrl_mode;
static ctrl_mode_e last_vision_flag;
uint8_t vision_flag;
static uint8_t s_user_mode_valid;

void mode_user_set(ctrl_mode_e mode)
{
    s_user_mode_valid = 1U;
    ctrl_mode = mode;
}

void mode_user_clear(void)
{
    s_user_mode_valid = 0U;
    ctrl_mode = PROTECT_MODE;
}

uint8_t mode_user_active(void)
{
    return s_user_mode_valid;
}

static void unlock_init(void)
{
    /* 无键盘: 仅接受左拨杆中、右拨杆上 + 内八。 */
    if (rc.sw1 == RC_MI && rc.sw2 == RC_UP) {
        if (rc.ch4 <= -600 && rc.ch3 >= 600 && rc.ch1 <= -600 && rc.ch2 <= -600) {
            lock_flag = 1;
        }
    }
}

static void sw1_mode_handler(void)
{
    last_vision_flag = ctrl_mode;
    switch (rc.sw1) {
        case RC_UP:
            if (rc.ch4 <= -600) {
                ctrl_mode = VISION_MODE;
                vision_mode = vMODE_AUTO;
            } else {
                ctrl_mode = REMOTER_MODE;
            }
            break;
        case RC_MI:
            ctrl_mode = PROTECT_MODE;
            break;
        case RC_DN:
            ctrl_mode = ARM_TEACH_MODE;  /* 无键盘,左拨杆下直接进入机械臂拖动示教 */
            break;
        default:
            ctrl_mode = PROTECT_MODE;
            break;
    }
    if (last_vision_flag == VISION_MODE && ctrl_mode != VISION_MODE) vision_flag = 1;
}

void Rc_Fail_mode_handler(void)
{
    if (!rc.kb.bit.CTRL) return;
    if (rc.kb.bit.R) ctrl_mode = (rc.ch4 <= -600) ? VISION_MODE : REMOTER_MODE;
    if (rc.kb.bit.S) ctrl_mode = PROTECT_MODE;
    if (rc.kb.bit.B) ctrl_mode = rc.mouse.r ? VISION_MODE : KEYBOARD_MODE;
}

void vtm_mode_handler(void) {}
void vtm_unlock_init(void) {}
void vtm_safe_lock(void) {}

void mode_switch_task(void const *argu)
{
    (void)argu;
    ctrl_mode = PROTECT_MODE;
    lock_flag = 0;
    RC_FAIL_KEYBOARD_FLAG = 0;
    dbus_init();

    for (;;) {
        dbus_process();

        /* 显式覆盖优先：无遥控器示教/调试时不被断联保护覆盖 */
        if (s_user_mode_valid) {
            wdg_user_set_bit(WDG_BIT_TASK_MODES);
            osDelay(5);
            continue;
        }

        if (!dbus_online()) {
            ctrl_mode = PROTECT_MODE;
            lock_flag = 0;
            RC_FAIL_KEYBOARD_FLAG = 1;
        } else if (!lock_flag) {
            ctrl_mode = PROTECT_MODE;
            unlock_init();
        } else {
            RC_FAIL_KEYBOARD_FLAG = 0;
            sw1_mode_handler();
        }

        wdg_user_set_bit(WDG_BIT_TASK_MODES);
        osDelay(5);
    }
}

void fly_led_status(void)
{
    if (rc.kb.bit.W || rc.ch4 > 600) {
        update_leds(RGB_NUM, OFF);
        led_range_on(4, 12, PURPLE);
    } else if (rc.kb.bit.D || rc.ch3 > 600) {
        update_leds(RGB_NUM, OFF);
        led_range_on(0, 8, PURPLE);
    } else if (rc.kb.bit.S || rc.ch4 < -600) {
        update_leds(RGB_NUM, OFF);
        led_range_on(0, 4, PURPLE);
        led_range_on(11, 16, PURPLE);
    } else if (rc.kb.bit.A || rc.ch3 < -600) {
        update_leds(RGB_NUM, OFF);
        led_range_on(8, 16, PURPLE);
    } else {
        led_double_flash(0, 16, PURPLE, 600, 600);
    }
    rgb_output_data();
}
