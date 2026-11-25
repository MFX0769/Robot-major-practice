#include "arm_task.h"
#include "main.h"
#include "prot_servo.h"
#include "prot_sbus.h"
#include "dbg_print.h"
#include "mode_switch_task.h"
#include "cmsis_os.h"
#include <stdlib.h>

int16_t test_sh_pulse = 1500;
int16_t test_el_pulse = 1500;
int16_t test_gr_pulse = 1500;
volatile uint8_t arm_busy = 0;

void arm_task_init(void)
{
    servo_init();
}

void arm_task(void const *argu)
{
    (void)argu;
    arm_task_init();

    for (;;) {
        servo_poll();
        if (servo_config_active()) {
            arm_busy = 0;
            servo_set_enabled(1);
            osDelay(ARM_TASK_PERIOD);
            continue;
        }
        if (ctrl_mode == PROTECT_MODE) {
            servo_teach_end();
            arm_busy = 0;
            servo_set_enabled(0);
            osDelay(ARM_TASK_PERIOD);
            continue;
        }
        if (ctrl_mode == ARM_TEACH_MODE) {
            if (!servo_teach_active()) {
                /* 先给舵机上电并等稳定, 再发 PRAD/PULK。 */
                servo_set_enabled(1);
                osDelay(500);
                if (servo_teach_begin() != 0U) {
                    dbg_print("[SERVO] teach_begin failed, motion inhibited\r\n");
                    servo_set_enabled(0);
                    osDelay(ARM_TASK_PERIOD);
                    continue;
                }
                dbg_print("[SERVO] teach_begin\r\n");
            }
            servo_teach_poll();
            arm_busy = 1;
            servo_set_enabled(1);
            osDelay(ARM_TASK_PERIOD);
            continue;
        }
        if (servo_teach_active()) {
            servo_teach_end();
            dbg_print("[SERVO] teach_end points=%u\r\n", servo_teach_count);
            arm_busy = 0;
        }
        if (ctrl_mode == REMOTER_MODE && wbus_rc.data.sw3) {

            if (wbus_rc.data.sw1 == 0) {
                arm_busy = 0;
                test_sh_pulse = 1500 + wbus_rc.data.ch1 * 500 / 1024;
                test_el_pulse = 1500 + wbus_rc.data.ch2 * 500 / 1024;
                test_gr_pulse = 1500 + wbus_rc.data.ch4 * 300 / 1024;
                servo_set_pulse(SERVO_SHOULDER, test_sh_pulse);
                servo_set_pulse(SERVO_ELBOW, test_el_pulse);
                servo_set_pulse(SERVO_GRIPPER, test_gr_pulse);
            } else {
                static uint8_t didx = 0;
                static uint32_t t0 = 0;
                if (t0 == 0) t0 = osKernelSysTick();
                if (abs(wbus_rc.data.ch2) > 200 && didx == 0) {
                    didx = 1;
                    t0 = osKernelSysTick();
                }
                if (osKernelSysTick() - t0 >= 2000 && didx > 0) {
                    didx = (didx + 1) % 6;
                    t0 = osKernelSysTick();
                }
                switch (didx) {
                    case 0: servo_set_pulse(SERVO_SHOULDER, 1333); servo_set_pulse(SERVO_ELBOW, 1500); servo_set_pulse(SERVO_GRIPPER, 1100); break;
                    case 1: servo_set_pulse(SERVO_SHOULDER, 1700); servo_set_pulse(SERVO_ELBOW, 2000); servo_set_pulse(SERVO_GRIPPER, 1100); break;
                    case 2: servo_set_pulse(SERVO_SHOULDER, 1700); servo_set_pulse(SERVO_ELBOW, 2000); servo_set_pulse(SERVO_GRIPPER, 1400); break;
                    case 3: servo_set_pulse(SERVO_SHOULDER, 1700); servo_set_pulse(SERVO_ELBOW, 2000); servo_set_pulse(SERVO_GRIPPER, 1450); break;
                    case 4: servo_set_pulse(SERVO_SHOULDER, 1333); servo_set_pulse(SERVO_ELBOW, 1500); servo_set_pulse(SERVO_GRIPPER, 1400); break;
                    case 5: servo_set_pulse(SERVO_SHOULDER, 1333); servo_set_pulse(SERVO_ELBOW, 1500); servo_set_pulse(SERVO_GRIPPER, 1100); break;
                }
                arm_busy = (didx >= 1 && didx <= 4);
            }
        } else {
            arm_busy = 0;
            servo_set_enabled(0);
        }

        osDelay(ARM_TASK_PERIOD);
    }
}

void arm_vision_callback(void) {}
