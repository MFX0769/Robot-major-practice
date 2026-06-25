#include "arm_task.h"
#include "main.h"
#include "bsp_pwm.h"
#include "bsp_gpio.h"
#include "prot_servo.h"
#include "prot_sbus.h"
#include "mode_switch_task.h"
#include "prot_rm_motor.h"

int16_t test_sh_pulse = 1500;
int16_t test_el_pulse = 1500;
int16_t test_gr_pulse = 1500;

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
        /* ������磺ȫ��ң��+SWD�򿪲�ͨ�� */
        if (ctrl_mode == REMOTER_MODE && wbus_rc.data.sw3) {
            bsp_gpio_servo_power_on();
        } else {
            bsp_gpio_servo_power_off();
        }

        /* PROTECT_MODE �� ȫ����λ */
        if (ctrl_mode == PROTECT_MODE) {
            __HAL_TIM_SET_COMPARE(&htim2, SERVO_SHOULDER_CHANNEL, 1500);
            __HAL_TIM_SET_COMPARE(&htim2, SERVO_ELBOW_CHANNEL, 1500);
            __HAL_TIM_SET_COMPARE(&htim3, SERVO_GRIPPER_CHANNEL, 1500);
            osDelay(ARM_TASK_PERIOD);
            continue;
        }

        /* SWB��+SWD�� �� �ֶ����� */
        if (wbus_rc.data.sw1 == 0 && wbus_rc.data.sw3) {
            test_sh_pulse = 1500 + wbus_rc.data.ch1 * 500 / 1024;
            test_el_pulse = 1500 + wbus_rc.data.ch2 * 500 / 1024;
            test_gr_pulse = 1500 + wbus_rc.data.ch4 * 300 / 1024;
            if (test_sh_pulse < 500 || test_sh_pulse > 2500) test_sh_pulse = 1500;
            if (test_el_pulse < 500 || test_el_pulse > 2500) test_el_pulse = 1500;
            if (test_gr_pulse < 800 || test_gr_pulse > 2200) test_gr_pulse = 1500;
            __HAL_TIM_SET_COMPARE(&htim2, SERVO_SHOULDER_CHANNEL, test_sh_pulse);
            __HAL_TIM_SET_COMPARE(&htim2, SERVO_ELBOW_CHANNEL, test_el_pulse);
            __HAL_TIM_SET_COMPARE(&htim3, SERVO_GRIPPER_CHANNEL, test_gr_pulse);
        } else if (ctrl_mode == REMOTER_MODE && wbus_rc.data.sw3) {
            /* ���ݣ�6֡ѭ�� */
            static uint8_t didx = 0;
            static uint32_t t0 = 0;
            if (t0 == 0) { t0 = osKernelSysTick(); }
            if (osKernelSysTick() - t0 >= 2000) { didx = (didx + 1) % 6; t0 = osKernelSysTick(); }
            switch (didx) {
                case 0: __HAL_TIM_SET_COMPARE(&htim2, SERVO_SHOULDER_CHANNEL, 1333); __HAL_TIM_SET_COMPARE(&htim2, SERVO_ELBOW_CHANNEL, 1500); __HAL_TIM_SET_COMPARE(&htim3, SERVO_GRIPPER_CHANNEL, 1100); break;
                case 1: __HAL_TIM_SET_COMPARE(&htim2, SERVO_SHOULDER_CHANNEL, 1700); __HAL_TIM_SET_COMPARE(&htim2, SERVO_ELBOW_CHANNEL, 2000); __HAL_TIM_SET_COMPARE(&htim3, SERVO_GRIPPER_CHANNEL, 1100); break;
                case 2: __HAL_TIM_SET_COMPARE(&htim2, SERVO_SHOULDER_CHANNEL, 1700); __HAL_TIM_SET_COMPARE(&htim2, SERVO_ELBOW_CHANNEL, 2000); __HAL_TIM_SET_COMPARE(&htim3, SERVO_GRIPPER_CHANNEL, 1400); break;
                case 3: __HAL_TIM_SET_COMPARE(&htim2, SERVO_SHOULDER_CHANNEL, 1700); __HAL_TIM_SET_COMPARE(&htim2, SERVO_ELBOW_CHANNEL, 2000); __HAL_TIM_SET_COMPARE(&htim3, SERVO_GRIPPER_CHANNEL, 1450); break;
                case 4: __HAL_TIM_SET_COMPARE(&htim2, SERVO_SHOULDER_CHANNEL, 1333); __HAL_TIM_SET_COMPARE(&htim2, SERVO_ELBOW_CHANNEL, 1500); __HAL_TIM_SET_COMPARE(&htim3, SERVO_GRIPPER_CHANNEL, 1400); break;
                case 5: __HAL_TIM_SET_COMPARE(&htim2, SERVO_SHOULDER_CHANNEL, 1333); __HAL_TIM_SET_COMPARE(&htim2, SERVO_ELBOW_CHANNEL, 1500); __HAL_TIM_SET_COMPARE(&htim3, SERVO_GRIPPER_CHANNEL, 1100); break;
            }
        } else {
            /* SWD�� �� ready */
            __HAL_TIM_SET_COMPARE(&htim2, SERVO_SHOULDER_CHANNEL, 1333);
            __HAL_TIM_SET_COMPARE(&htim2, SERVO_ELBOW_CHANNEL, 1500);
            __HAL_TIM_SET_COMPARE(&htim3, SERVO_GRIPPER_CHANNEL, 1500);
        }
        osDelay(ARM_TASK_PERIOD);
    }
}

void arm_vision_callback(void) {}

