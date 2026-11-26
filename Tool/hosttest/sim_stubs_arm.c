/* 只给机械臂/按键测试用：那里不编真的 servo_test.c，控制台函数用空实现顶上。 */
#include <stdint.h>
#include "servo_test.h"
#include "prot_sbus.h"
#include "chassis_task.h"

uint8_t s_config_allowed_for_arm_test;

/* 真固件里定义在 prot_sbus.c；这两个测试不编它 */
wbus_frame_u wbus_rc;

/* 机械臂测试不编 chassis_task.c：方向微调只记不生效 */
void chassis_map_sign_set(uint8_t idx, int8_t sign) { (void)idx; (void)sign; }

void servo_test_init(void) {}
void servo_test_process(void) {}
void servo_test_poll(void) {}
void servo_test_set_config_allowed(uint8_t allowed) { s_config_allowed_for_arm_test = allowed; }
uint8_t servo_test_config_active(void) { return s_config_allowed_for_arm_test; }
