/* 只给控制台/底盘/DBUS 测试用：真的 servo_test.c、prot_sbus.c 会引用这些，
   而那里不编 舵机驱动、巡线、W25Q64、DR16、bsp_can。 */
#include <string.h>
#include "sim.h"
#include "main.h"
#include "bsp_can.h"
#include "prot_dbus.h"
#include "prot_sbus.h"
#include "prot_dr16.h"
#include "prot_xb_servo.h"
#include "prot_servo.h"
#include "prot_8irtraking.h"

static bsp_can_stat_t s_can[2];
volatile uint32_t ir_isr_bytes;
uint8_t lock_flag;                     /* 真固件里在 mode_switch_task.c；测试里只有 rc 命令打印它 */
/* 初值和真驱动一致：肩 ID=002，肘 ID=001 */
xb_servo_dev_t xb_servo[XB_JOINT_NUM] = {
    {2, 0, XB_PULSE_ZERO, 0, 0, 0},
    {1, 0, XB_PULSE_ZERO, 0, 0, 0},
};
uint32_t g_wdg_bits;
uint32_t g_sbus_calls;
uint32_t g_w25_erase_calls;
uint32_t g_xb_raw_calls;

/* ---- bsp_can ---- */
void bsp_can_stat_get(can_channel_e ch, bsp_can_stat_t *out)
{
    if (out != NULL) *out = s_can[(ch == CAN_CHANNEL_1) ? 0U : 1U];
}

void bsp_can_recover(can_channel_e ch)
{
    s_can[(ch == CAN_CHANNEL_1) ? 0U : 1U].bo = 0U;
}

/* 测试直接摆一个 CAN 状态出来，验证 can stat / chassis status 的打印路径 */
void sim_can_set(int ch, uint32_t tx, uint32_t rx, uint8_t tec, uint8_t bo, uint32_t off_total)
{
    s_can[(ch == 1) ? 0U : 1U].tx_ok = tx;
    s_can[(ch == 1) ? 0U : 1U].rx = rx;
    s_can[(ch == 1) ? 0U : 1U].tec = tec;
    s_can[(ch == 1) ? 0U : 1U].bo = bo;
    s_can[(ch == 1) ? 0U : 1U].bus_off = off_total;
}

/* ---- DBUS 查询（servo_test.c 的 uart 命令用）。真的 prot_dbus.c 也定义了这几个，
   所以 test_dbus 不编本文件的这一段：用 SIM_NO_DBUS_STUBS 区分 ---- */
#ifndef SIM_NO_DBUS_STUBS
static uint32_t s_dbus_bytes, s_dbus_frames;
static uint8_t s_dbus_online;
uint32_t dbus_rx_bytes(void) { return s_dbus_bytes; }
uint32_t dbus_rx_count(void) { return s_dbus_frames; }
uint8_t dbus_online(void) { return s_dbus_online; }
void sim_dbus_set(uint32_t bytes, uint32_t frames, uint8_t online)
{
    s_dbus_bytes = bytes;
    s_dbus_frames = frames;
    s_dbus_online = online;
}
#endif

void wdg_user_set_bit(uint32_t bit) { g_wdg_bits |= bit; }

/* ---- DR16 解码（真文件依赖遥控融合模块，这里只按测试要求给出解码结果） ---- */
rc_info_t rc;
static uint8_t s_sw2_to_deliver;
uint8_t g_rc_last[18];

/* 测试指定下一次解码出来的右拨杆位置（DBUS 编码：1 上 2 下 3 中） */
void sim_rc_set_sw2(uint8_t dbus_sw2) { s_sw2_to_deliver = dbus_sw2; }

void rc_callback_handler(rc_info_t *r, uint8_t *buff)
{
    g_sbus_calls++;
    memcpy(g_rc_last, buff, 18U);           /* 解码函数拿到的原始 18 字节，测试拿去和发出来的逐字节比 */
    r->sw1 = 3U;
    r->sw2 = s_sw2_to_deliver;
}

/* ---- 舵机驱动的原始接口 ---- */
uint8_t xb_servo_ping_raw(uint8_t id) { g_xb_raw_calls++; (void)id; return 0U; }
const char *xb_servo_last_frame(void) { return "#000PID002!"; }
uint8_t xb_servo_read_id_raw(uint8_t servo_id, uint8_t *id) { g_xb_raw_calls++; *id = servo_id; return 0U; }
uint8_t xb_servo_write_id_raw(uint8_t old_id, uint8_t new_id)
{
    g_xb_raw_calls++;
    xb_servo[0].id = (uint8_t)(old_id + new_id);       /* 留个痕迹，测试能看到 setid 真的被执行了 */
    return 0U;
}
uint8_t xb_servo_bind(uint8_t joint, uint8_t id) { g_xb_raw_calls++; xb_servo[joint].id = id; return 0U; }
uint32_t xb_servo_rx_errors(void) { return 7U; }
uint32_t xb_servo_rx_bytes(void) { return 1234U; }

/* ---- 舵机统一接口里控制台会用到、sim.c 没有的那一个（和真函数一样：进配置模式才允许 setid） ---- */
#ifndef SIM_NO_SERVO_STUBS
#include "servo_test.h"
void servo_set_config_mode(uint8_t enabled)
{
    g_config_active = (enabled != 0U) ? 1U : 0U;
    servo_test_set_config_allowed(enabled);
}
#endif

/* ---- W25Q64 ---- */
int8_t OSPI_W25Qxx_BlockErase_32K(uint32_t addr) { (void)addr; g_w25_erase_calls++; return 0; }
int8_t OSPI_W25Qxx_WriteBuffer(uint8_t *buf, uint32_t addr, uint32_t size) { (void)buf; (void)addr; (void)size; return 0; }
int8_t OSPI_W25Qxx_ReadBuffer(uint8_t *buf, uint32_t addr, uint32_t size)
{
    uint32_t i;
    (void)addr;
    for (i = 0U; i < size; i++) buf[i] = (uint8_t)(0x5AU + i * 7U);     /* 假装写进去的数据原样读回来 */
    return 0;
}
uint32_t OSPI_W25Qxx_ReadID(void) { return 0xEF4017UL; }
