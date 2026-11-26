/* 只给机械臂/按键测试用：那里不编真的 servo_test.c，控制台函数用空实现顶上。 */
#include <stdint.h>
#include "servo_test.h"
#include "prot_sbus.h"
#include "chassis_task.h"
#include "pid.h"
#include "prot_dbus.h"
#include "prot_8irtraking.h"
#include "usb_comm.h"

uint8_t s_config_allowed_for_arm_test;

/* 真固件里定义在 prot_sbus.c；这两个测试不编它 */
wbus_frame_u wbus_rc;

/* 机械臂测试不编 chassis_task.c：方向微调和避障标定只吞参数，不生效 */
void chassis_map_sign_set(uint8_t idx, int8_t sign) { (void)idx; (void)sign; }
static pid_t s_arm_test_line_pid;
pid_t *chassis_dbg_line_pid_get(void) { return &s_arm_test_line_pid; }
void chassis_av_apply(uint16_t kodo, uint16_t shift, uint16_t fwd, int8_t side)
{
    (void)kodo; (void)shift; (void)fwd; (void)side;
}

void servo_test_init(void) {}
void servo_test_process(void) {}
void servo_test_poll(void) {}
void servo_test_set_config_allowed(uint8_t allowed) { s_config_allowed_for_arm_test = allowed; }
uint8_t servo_test_config_active(void) { return s_config_allowed_for_arm_test; }
void servo_test_stats_get(uint32_t *bytes, uint32_t *lines, uint32_t *enq, uint32_t *drop)
{
    if (bytes != NULL) *bytes = 0U;
    if (lines != NULL) *lines = 0U;
    if (enq != NULL) *enq = 0U;
    if (drop != NULL) *drop = 0U;
}
uint8_t lock_flag;

/* 机械臂/按键测试不编这些任务：诊断接口给确定值即可 */
void dbus_stats_get(uint32_t *frames, uint32_t *bad, uint32_t *drop, uint32_t *last_ms, uint32_t *max_ms)
{
    if (frames != NULL) *frames = 0U;
    if (bad != NULL) *bad = 0U;
    if (drop != NULL) *drop = 0U;
    if (last_ms != NULL) *last_ms = 0U;
    if (max_ms != NULL) *max_ms = 0U;
}
void usb_tx_stats_get(uint32_t *enq, uint32_t *drop, uint32_t *tx, uint32_t *busy_drop)
{
    if (enq != NULL) *enq = 0U;
    if (drop != NULL) *drop = 0U;
    if (tx != NULL) *tx = 0U;
    if (busy_drop != NULL) *busy_drop = 0U;
}
prot_8irtraking_data_t s_arm_test_line;
uint32_t dbus_rx_count(void) { return 0U; }
uint8_t dbus_online(void) { return 0U; }
