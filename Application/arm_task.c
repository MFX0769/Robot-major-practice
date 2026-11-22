#include "arm_task.h"
#include "main.h"
#include "bsp_pwm.h"
#include "bsp_gpio.h"
#include "prot_servo.h"
#include "prot_sbus.h"
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
    bsp_pwm_start();
    bsp_gpio_servo_power_off();
}

void arm_task(void)
{
    arm_task_init();
    for (;;) {
        if (ctrl_mode == REMOTER_MODE && wbus_rc.data.sw3) {
            bsp_gpio_servo_power_on();
        } else {
            bsp_gpio_servo_power_off();
        }

        if (ctrl_mode == PROTECT_MODE) {
            arm_busy = 0;
            servo_set_pulse(SERVO_SHOULDER,1500);
            servo_set_pulse(SERVO_ELBOW,1500);
            servo_set_pulse(SERVO_GRIPPER,1500);
            osDelay(ARM_TASK_PERIOD);
            continue;
        }

        /* SWB�?SWD�?= 手动调参 */
        if (wbus_rc.data.sw1 == 0 && wbus_rc.data.sw3) {
            arm_busy = 0;
            test_sh_pulse = 1500 + wbus_rc.data.ch1 * 500 / 1024;
            test_el_pulse = 1500 + wbus_rc.data.ch2 * 500 / 1024;
            test_gr_pulse = 1500 + wbus_rc.data.ch4 * 300 / 1024;
            servo_set_pulse(SERVO_SHOULDER,test_sh_pulse);
            servo_set_pulse(SERVO_ELBOW,test_el_pulse);
            servo_set_pulse(SERVO_GRIPPER,test_gr_pulse);

        /* SWD�?= 表演 */
        } else if (ctrl_mode == REMOTER_MODE && wbus_rc.data.sw3) {
            static uint8_t didx = 0;
            static uint32_t t0 = 0;
            if (t0 == 0) t0 = osKernelSysTick();
            if (abs(wbus_rc.data.ch2) > 200)
                if (didx == 0) { didx = 1; t0 = osKernelSysTick(); }
            if (osKernelSysTick() - t0 >= 2000 && didx > 0) { didx = (didx + 1) % 6; t0 = osKernelSysTick(); }
            switch (didx) {
                case 0: servo_set_pulse(SERVO_SHOULDER,1333); servo_set_pulse(SERVO_ELBOW,1500); servo_set_pulse(SERVO_GRIPPER,1100); break;
                case 1: servo_set_pulse(SERVO_SHOULDER,1700); servo_set_pulse(SERVO_ELBOW,2000); servo_set_pulse(SERVO_GRIPPER,1100); break;
                case 2: servo_set_pulse(SERVO_SHOULDER,1700); servo_set_pulse(SERVO_ELBOW,2000); servo_set_pulse(SERVO_GRIPPER,1400); break;
                case 3: servo_set_pulse(SERVO_SHOULDER,1700); servo_set_pulse(SERVO_ELBOW,2000); servo_set_pulse(SERVO_GRIPPER,1450); break;
                case 4: servo_set_pulse(SERVO_SHOULDER,1333); servo_set_pulse(SERVO_ELBOW,1500); servo_set_pulse(SERVO_GRIPPER,1400); break;
                case 5: servo_set_pulse(SERVO_SHOULDER,1333); servo_set_pulse(SERVO_ELBOW,1500); servo_set_pulse(SERVO_GRIPPER,1100); break;
            }
            arm_busy = (didx >= 1 && didx <= 4);
        } else {
            arm_busy = 0;
            servo_set_pulse(SERVO_SHOULDER,1333);
            servo_set_pulse(SERVO_ELBOW,1500);
            servo_set_pulse(SERVO_GRIPPER,1500);
        }
        osDelay(ARM_TASK_PERIOD);
    }
}

void arm_vision_callback(void) {}
