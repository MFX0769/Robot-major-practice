/**
  * @file    arm_task.c
  * @brief   机械臂：yaw 双环 + 单按钮拖动示教 + 四关节点动
  * @note    只改机械臂相关逻辑；CAN 发送仍走 dji_send_current -> CAN2 队列。
  */
#include "arm_task.h"
#include "arm_yaw_limit.h"
#include "prot_servo.h"
#include "servo_test.h"
#include "prot_xb_servo.h"
#include "prot_rm_motor.h"
#include "mode_switch_task.h"
#include "bsp_user_key.h"
#include "prot_vision.h"
#include "w25q64.h"
#include "dbg_print.h"
#include "cmsis_os.h"
#include "pid.h"
#include <string.h>

/* ========================= 常量 ========================= */
#define ARM_YAW_DEG_PER_ECD   (360.0f / 8192.0f)

#define TEACH_FLASH_A         0x000000UL
#define TEACH_FLASH_B         0x008000UL
#define TEACH_MAGIC           0x41544C31UL   /* "ATL1" */
#define TEACH_VER             1U

#define TEACH_DYAW_ECD        4
#define TEACH_DSH_US          5
#define TEACH_DEL_US          5
#define TEACH_DGR_US          10

/* ========================= 全局 ========================= */
arm_ctrl_t arm_ctrl;
arm_teach_point_t arm_teach_points[ARM_TEACH_MAX_POINTS];
uint16_t arm_teach_count;
volatile uint8_t arm_busy;

static uint8_t  s_flash_ok;
static uint32_t s_flash_seq;
static arm_teach_point_t s_flash_tmp[ARM_TEACH_MAX_POINTS];
static arm_teach_point_t s_last_point;

static int32_t  s_last_rx_cnt;
static uint32_t s_last_rx_tick;
static uint32_t s_last_send_tick;
static uint32_t s_last_report_tick;
static uint8_t  s_yaw_report_force;

static uint8_t  s_jog_active;
static uint32_t s_jog_tick;
static uint8_t  s_motion_locked = 1U;
static uint8_t  s_target_valid;      /* 0=目标跟随当前 ECD，避免上电首次使能猛冲 */
static uint8_t  s_manual_override;   /* 控制台显式动作，允许在 PROTECT 下调试 */
static uint32_t s_yaw_cmd_tick;      /* yaw 命令保持时间，超时归零 */

static void teach_abort(const char *why);

/* ========================= 小工具 ========================= */
static uint32_t now_ms(void)
{
    return osKernelSysTick();
}

static int32_t clamp_ecd(int32_t ecd)
{
    if (ecd < ARM_YAW_ECD_LEFT_DEFAULT) ecd = ARM_YAW_ECD_LEFT_DEFAULT;
    if (ecd > ARM_YAW_ECD_RIGHT_DEFAULT) ecd = ARM_YAW_ECD_RIGHT_DEFAULT;
    return ecd;
}

static uint32_t crc32_calc(const uint8_t *p, uint32_t n)
{
    uint32_t c = 0xFFFFFFFFUL;
    while (n-- > 0U) {
        uint8_t i;
        c ^= *p++;
        for (i = 0; i < 8U; i++) {
            c = (c >> 1) ^ (0xEDB88320UL & (0U - (c & 1U)));
        }
    }
    return ~c;
}

const char *arm_teach_state_name(void)
{
    switch (arm_ctrl.teach.state) {
        case ARM_TEACH_IDLE:     return "IDLE";
        case ARM_TEACH_READY:    return "READY";
        case ARM_TEACH_RECORD:   return "RECORD";
        case ARM_TEACH_SAVED:    return "SAVED";
        case ARM_TEACH_PLAYBACK: return "PLAYBACK";
        case ARM_TEACH_ABORT:    return "ABORT";
        default:                 return "?";
    }
}

uint8_t arm_teach_is_recording(void)
{
    return (arm_ctrl.teach.state == ARM_TEACH_RECORD) ? 1U : 0U;
}

uint8_t arm_flash_ready(void)
{
    return s_flash_ok;
}

/* ========================= 片内 Flash 轨迹 ========================= */
typedef struct {
    uint32_t magic;
    uint32_t seq;
    uint16_t count;
    uint16_t ver;
    uint32_t crc;
} teach_hdr_t;

static int flash_save(void)
{
    teach_hdr_t hdr;
    uint32_t addr;
    int8_t rc;

    if (!s_flash_ok || arm_teach_count < 2U) return -1;

    s_flash_seq++;
    hdr.magic = TEACH_MAGIC;
    hdr.seq   = s_flash_seq;
    hdr.count = arm_teach_count;
    hdr.ver   = TEACH_VER;
    hdr.crc   = crc32_calc((const uint8_t *)arm_teach_points,
                           (uint32_t)arm_teach_count * sizeof(arm_teach_point_t));

    addr = ((s_flash_seq & 1U) != 0U) ? TEACH_FLASH_B : TEACH_FLASH_A;

    rc = OSPI_W25Qxx_BlockErase_32K(addr);
    if (rc != OSPI_W25Qxx_OK) return -2;
    rc = OSPI_W25Qxx_WriteBuffer((uint8_t *)&hdr, addr, sizeof(hdr));
    if (rc != OSPI_W25Qxx_OK) return -3;
    rc = OSPI_W25Qxx_WriteBuffer((uint8_t *)arm_teach_points,
                                 addr + sizeof(hdr),
                                 (uint32_t)arm_teach_count * sizeof(arm_teach_point_t));
    if (rc != OSPI_W25Qxx_OK) return -4;

    memset(&hdr, 0, sizeof(hdr));
    (void)OSPI_W25Qxx_ReadBuffer((uint8_t *)&hdr, addr, sizeof(hdr));
    if (hdr.magic != TEACH_MAGIC || hdr.count != arm_teach_count) return -5;

    (void)OSPI_W25Qxx_ReadBuffer((uint8_t *)s_flash_tmp, addr + sizeof(hdr),
                                 (uint32_t)arm_teach_count * sizeof(arm_teach_point_t));
    if (crc32_calc((const uint8_t *)s_flash_tmp,
                   (uint32_t)arm_teach_count * sizeof(arm_teach_point_t)) != hdr.crc) return -6;
    return 0;
}

static int flash_load(void)
{
    teach_hdr_t hdr;
    uint32_t best_seq = 0U;
    uint8_t found = 0U;
    const uint32_t addrs[2] = { TEACH_FLASH_A, TEACH_FLASH_B };
    uint8_t i;

    if (!s_flash_ok) return -1;

    for (i = 0U; i < 2U; i++) {
        memset(&hdr, 0, sizeof(hdr));
        if (OSPI_W25Qxx_ReadBuffer((uint8_t *)&hdr, addrs[i], sizeof(hdr)) != OSPI_W25Qxx_OK) continue;
        if (hdr.magic != TEACH_MAGIC || hdr.ver != TEACH_VER) continue;
        if (hdr.count < 2U || hdr.count > ARM_TEACH_MAX_POINTS) continue;
        if (OSPI_W25Qxx_ReadBuffer((uint8_t *)s_flash_tmp, addrs[i] + sizeof(hdr),
                                   (uint32_t)hdr.count * sizeof(arm_teach_point_t)) != OSPI_W25Qxx_OK) continue;
        if (crc32_calc((const uint8_t *)s_flash_tmp,
                       (uint32_t)hdr.count * sizeof(arm_teach_point_t)) != hdr.crc) continue;
        if (!found || hdr.seq > best_seq) {
            memcpy(arm_teach_points, s_flash_tmp,
                   (uint32_t)hdr.count * sizeof(arm_teach_point_t));
            arm_teach_count = hdr.count;
            best_seq = hdr.seq;
            found = 1U;
        }
    }

    if (found) {
        s_flash_seq = best_seq;
        arm_ctrl.teach.saved_valid = 1U;
        return 0;
    }
    return -2;
}

static void flash_erase_all(void)
{
    if (!s_flash_ok) return;
    (void)OSPI_W25Qxx_BlockErase_32K(TEACH_FLASH_A);
    (void)OSPI_W25Qxx_BlockErase_32K(TEACH_FLASH_B);
    s_flash_seq = 0U;
}

/* ========================= 示教状态机 ========================= */
static void teach_set_state(arm_teach_state_e s, const char *why)
{
    if (arm_ctrl.teach.state != s) {
        dbg_print("[TEACH] %s -> %s (%s) pts=%u\r\n",
                  arm_teach_state_name(), (s == ARM_TEACH_IDLE) ? "IDLE" :
                  (s == ARM_TEACH_READY) ? "READY" :
                  (s == ARM_TEACH_RECORD) ? "RECORD" :
                  (s == ARM_TEACH_SAVED) ? "SAVED" :
                  (s == ARM_TEACH_PLAYBACK) ? "PLAYBACK" : "ABORT",
                  (why != NULL) ? why : "", (unsigned)arm_teach_count);
    }
    arm_ctrl.teach.state = s;
    arm_ctrl.teach.state_tick = now_ms();
    arm_busy = (s != ARM_TEACH_IDLE) ? 1U : 0U;   /* 示教会话期间底盘必须锁零 */
}

static int teach_read_joints(uint16_t *sh, uint16_t *el, uint16_t *gr, int16_t *yaw)
{
    if (xb_servo_read_pos(0U, sh) != 0U) return -1;
    if (xb_servo_read_pos(1U, el) != 0U) return -2;
    *gr  = servo_get_current_pulse(2U);
    *yaw = (int16_t)arm_yaw_motor.ecd;
    return 0;
}

static void teach_add_point(uint8_t force)
{
    arm_teach_point_t p;
    int rc;

    if (arm_teach_count >= ARM_TEACH_MAX_POINTS) {
        teach_abort("full");
        return;
    }

    rc = teach_read_joints(&p.shoulder, &p.elbow, &p.gripper, &p.yaw_ecd);
    if (rc != 0) {
        dbg_print("[TEACH] servo offline rc=%d -> abort\r\n", rc);
        teach_abort("servo offline");
        return;
    }

    if (!force && arm_teach_count > 0U) {
        int32_t dy = (int32_t)p.yaw_ecd - (int32_t)s_last_point.yaw_ecd;
        int32_t ds = (int32_t)p.shoulder - (int32_t)s_last_point.shoulder;
        int32_t de = (int32_t)p.elbow - (int32_t)s_last_point.elbow;
        int32_t dg = (int32_t)p.gripper - (int32_t)s_last_point.gripper;
        if (dy < 0) dy = -dy;
        if (ds < 0) ds = -ds;
        if (de < 0) de = -de;
        if (dg < 0) dg = -dg;
        if (dy < TEACH_DYAW_ECD && ds < TEACH_DSH_US &&
            de < TEACH_DEL_US && dg < TEACH_DGR_US) {
            return;
        }
    }

    arm_teach_points[arm_teach_count++] = p;
    s_last_point = p;
    if (force) {
        dbg_print("[TEACH] key pt=%u yaw=%d sh=%u el=%u gr=%u\r\n",
                  (unsigned)arm_teach_count, (int)p.yaw_ecd,
                  (unsigned)p.shoulder, (unsigned)p.elbow, (unsigned)p.gripper);
    }
}

static void teach_enter(void)
{
    if (arm_ctrl.teach.state == ARM_TEACH_READY ||
        arm_ctrl.teach.state == ARM_TEACH_RECORD ||
        arm_ctrl.teach.state == ARM_TEACH_PLAYBACK) {
        return;
    }

    mode_user_set(ARM_TEACH_MODE);
    arm_yaw_set_enable(0U);     /* 先断 yaw 转矩，再释力肩/肘 */
    servo_set_enabled(1U);
    if (servo_teach_begin() != 0U) {
        dbg_print("[TEACH] begin failed (servo offline?) -> ABORT\r\n");
        teach_set_state(ARM_TEACH_ABORT, "begin failed");
        mode_user_clear();
        return;
    }

    arm_ctrl.yaw.enable = 0U;
    arm_yaw_set_enable(0U);

    arm_teach_count = 0U;
    arm_ctrl.teach.count = 0U;
    arm_ctrl.teach.origin_valid = 0U;
    arm_ctrl.teach.origin_ecd = 0;
    arm_ctrl.teach.saved_valid = 0U;
    arm_ctrl.teach.play_index = 0U;
    arm_ctrl.teach.delete_confirm = 0U;
    arm_ctrl.teach.play_max_err = 0;
    s_motion_locked = 0U;
    s_manual_override = 1U;
    teach_set_state(ARM_TEACH_READY, "long2");
    dbg_print("[TEACH] released. short=origin/record/key, double=save/play, long5=erase\r\n");
}

static void teach_exit(void)
{
    servo_teach_end();
    servo_set_enabled(1U);
    arm_yaw_set_enable(0U);
    arm_yaw_motor.tx_current = 0;
    s_motion_locked = 1U;
    s_manual_override = 0U;
    mode_user_clear();
    teach_set_state(ARM_TEACH_IDLE, "exit");
}

static void teach_abort(const char *why)
{
    servo_teach_end();
    servo_set_enabled(1U);
    arm_yaw_set_enable(0U);
    arm_yaw_motor.tx_current = 0;
    s_motion_locked = 1U;
    s_manual_override = 0U;
    mode_user_clear();
    teach_set_state(ARM_TEACH_ABORT, why);
}

static void teach_save(void)
{
    int rc;

    if (arm_teach_count < 2U) {
        dbg_print("[TEACH] save rejected: need >=2 pts (now=%u)\r\n",
                  (unsigned)arm_teach_count);
        return;
    }

    servo_teach_end();          /* 锁住肩/肘，防止保存后掉落 */
    servo_set_enabled(1U);
    arm_ctrl.yaw.target_ecd = (int32_t)arm_yaw_motor.ecd;
    s_target_valid = 0U;        /* 下次使能时同步到当前 ECD */
    arm_yaw_set_enable(0U);

    if (s_flash_ok) {
        rc = flash_save();
        dbg_print("[TEACH] saved %u pts, flash rc=%d seq=%lu\r\n",
                  (unsigned)arm_teach_count, rc, (unsigned long)s_flash_seq);
        if (rc != 0) {
            dbg_print("[TEACH] flash save failed, RAM copy kept\r\n");
        }
    } else {
        dbg_print("[TEACH] saved %u pts to RAM only (flash not ready)\r\n",
                  (unsigned)arm_teach_count);
    }
    arm_ctrl.teach.saved_valid = 1U;
    teach_set_state(ARM_TEACH_SAVED, "save");
}

static void teach_play_apply(uint16_t idx)
{
    const arm_teach_point_t *p;
    if (idx >= arm_teach_count) return;
    p = &arm_teach_points[idx];
    (void)servo_set_pulse(0U, p->shoulder);
    (void)servo_set_pulse(1U, p->elbow);
    (void)servo_set_pulse(2U, p->gripper);
    arm_ctrl.yaw.target_ecd = clamp_ecd((int32_t)p->yaw_ecd);
    arm_ctrl.yaw.enable = 1U;
    s_target_valid = 1U;
    s_motion_locked = 0U;
}

static void teach_play_start(void)
{
    if (arm_teach_count < 2U || !arm_ctrl.teach.saved_valid) {
        dbg_print("[TEACH] play rejected: no saved track\r\n");
        return;
    }
    if (!servo_online(0U) || !servo_online(1U)) {
        dbg_print("[TEACH] play rejected: shoulder/elbow offline\r\n");
        return;
    }
    servo_set_enabled(1U);
    servo_teach_end();          /* 回放时肩/肘必须锁力 */
    mode_user_set(ARM_TEACH_MODE);
    s_manual_override = 1U;
    arm_ctrl.teach.play_index = 0U;
    arm_ctrl.teach.play_max_err = 0;
    arm_ctrl.teach.last_sample_tick = now_ms();
    teach_set_state(ARM_TEACH_PLAYBACK, "double");
    teach_play_apply(0U);
    dbg_print("[TEACH] playback start, %u pts @20Hz\r\n", (unsigned)arm_teach_count);
}

static void teach_play_stop(void)
{
    arm_yaw_set_enable(0U);
    arm_yaw_motor.tx_current = 0;
    servo_set_enabled(1U);
    dbg_print("[TEACH] play done, max_yaw_err=%ld ECD\r\n",
              (long)arm_ctrl.teach.play_max_err);
    teach_set_state(ARM_TEACH_SAVED, "play done");
}

static void teach_erase(void)
{
    /* 任意示教态删除都必须先回安全态，避免伺服保持释力而底盘解锁 */
    if (arm_ctrl.teach.state != ARM_TEACH_IDLE) {
        servo_teach_end();
        servo_set_enabled(1U);
        arm_yaw_set_enable(0U);
        arm_yaw_motor.tx_current = 0;
        s_motion_locked = 1U;
        s_manual_override = 0U;
        mode_user_clear();
    }
    arm_teach_count = 0U;
    arm_ctrl.teach.count = 0U;
    arm_ctrl.teach.saved_valid = 0U;
    arm_ctrl.teach.origin_valid = 0U;
    arm_ctrl.teach.delete_confirm = 0U;
    flash_erase_all();
    arm_busy = 0U;
    teach_set_state(ARM_TEACH_IDLE, "erased");
    dbg_print("[TEACH] track erased (RAM+flash)\r\n");
}

static void teach_tick(void)
{
    uint32_t now = now_ms();
    arm_ctrl.teach.count = arm_teach_count;

    if (arm_ctrl.teach.delete_confirm &&
        (int32_t)(now - arm_ctrl.teach.delete_deadline) >= 0) {
        arm_ctrl.teach.delete_confirm = 0U;
        dbg_print("[TEACH] delete confirm timeout\r\n");
    }

    if (arm_ctrl.teach.state == ARM_TEACH_RECORD) {
        if ((uint32_t)(now - arm_ctrl.teach.last_sample_tick) >= ARM_TEACH_SAMPLE_MS) {
            arm_ctrl.teach.last_sample_tick = now;
            teach_add_point(0U);
        }
    } else if (arm_ctrl.teach.state == ARM_TEACH_PLAYBACK) {
        if ((uint32_t)(now - arm_ctrl.teach.last_sample_tick) >= ARM_TEACH_SAMPLE_MS) {
            arm_ctrl.teach.last_sample_tick = now;
            if (arm_ctrl.teach.play_index >= arm_teach_count) {
                teach_play_stop();
            } else {
                teach_play_apply(arm_ctrl.teach.play_index);
                arm_ctrl.teach.play_index++;
            }
        }
    }
}

static void on_short(void)
{
    if (arm_ctrl.teach.state == ARM_TEACH_READY) {
        if (!arm_ctrl.teach.origin_valid) {
            arm_ctrl.teach.origin_valid = 1U;
            arm_ctrl.teach.origin_ecd = (int32_t)arm_yaw_motor.ecd;
            dbg_print("[TEACH] origin ecd=%ld, short again to start record\r\n",
                      (long)arm_ctrl.teach.origin_ecd);
        } else {
            arm_teach_count = 0U;
            s_last_point.yaw_ecd = (int16_t)arm_yaw_motor.ecd;
            s_last_point.shoulder = servo_get_current_pulse(0U);
            s_last_point.elbow = servo_get_current_pulse(1U);
            s_last_point.gripper = servo_get_current_pulse(2U);
            arm_ctrl.teach.last_sample_tick = now_ms();
            teach_set_state(ARM_TEACH_RECORD, "record");
        }
    } else if (arm_ctrl.teach.state == ARM_TEACH_RECORD) {
        teach_add_point(1U);
    } else if (arm_ctrl.teach.state == ARM_TEACH_PLAYBACK) {
        teach_play_stop();
    } else if (arm_ctrl.teach.state == ARM_TEACH_SAVED) {
        dbg_print("[TEACH] short ignored in SAVED (double=play, long2=exit)\r\n");
    }
}

static void on_double(void)
{
    if (arm_ctrl.teach.state == ARM_TEACH_RECORD) {
        teach_save();
    } else if (arm_ctrl.teach.state == ARM_TEACH_SAVED) {
        teach_play_start();
    } else if (arm_ctrl.teach.state == ARM_TEACH_PLAYBACK) {
        teach_play_stop();
    } else if (arm_ctrl.teach.state == ARM_TEACH_IDLE ||
               arm_ctrl.teach.state == ARM_TEACH_ABORT) {
        /* 只允许回放“已保存”的轨迹；录音中断产生的半截 RAM 数据不得回放 */
        if (!arm_ctrl.teach.saved_valid) {
            int rc = flash_load();
            dbg_print("[TEACH] load flash rc=%d pts=%u\r\n", rc, (unsigned)arm_teach_count);
            if (rc != 0) {
                dbg_print("[TEACH] no saved track (unsaved pts=%u)\r\n",
                          (unsigned)arm_teach_count);
                return;
            }
        }
        if (arm_teach_count >= 2U) {
            arm_ctrl.teach.saved_valid = 1U;
            teach_set_state(ARM_TEACH_SAVED, "load");
        }
    }
}

static void on_long2(void)
{
    if (arm_ctrl.teach.state == ARM_TEACH_IDLE ||
        arm_ctrl.teach.state == ARM_TEACH_ABORT) {
        teach_enter();
    } else {
        teach_exit();
    }
}

static void on_long5(void)
{
    if (arm_ctrl.teach.state != ARM_TEACH_SAVED) {
        dbg_print("[TEACH] long5 ignored in %s\r\n", arm_teach_state_name());
        return;
    }
    if (!arm_ctrl.teach.delete_confirm) {
        arm_ctrl.teach.delete_confirm = 1U;
        arm_ctrl.teach.delete_deadline = now_ms() + 5000U;
        dbg_print("[TEACH] delete? hold 5s again within 5s to confirm\r\n");
    } else {
        teach_erase();
    }
}

static void handle_key(key_event_e ev)
{
    switch (ev) {
        case KEY_EVT_SHORT:  on_short();  break;
        case KEY_EVT_DOUBLE: on_double(); break;
        case KEY_EVT_TRIPLE: dbg_print("[KEY] triple\r\n"); break;
        case KEY_EVT_LONG2:  on_long2();  break;
        case KEY_EVT_LONG5:  on_long5();  break;
        default: break;
    }
}

/* ========================= yaw 双环 ========================= */
static void yaw_pid_init(void)
{
    /* 参数沿用 155ab05（6-25 旧双环），本轮只搭环不整定 */
    PID_struct_init(&arm_ctrl.yaw.angle_pid, 500U, 200U,
                    40.0f, 0.0f, 5.0f, 0.0f, 0.0f, 0.0f, NONE_IMPROVE);
    PID_struct_init(&arm_ctrl.yaw.spd_pid, 25000U, 10000U,
                    198.0f, 0.0f, 13.0f, 0.0f, 0.0f, 0.0f, NONE_IMPROVE);
    arm_ctrl.yaw.out_limit = ARM_YAW_OUT_LIMIT;
    arm_ctrl.yaw.target_ecd = (int32_t)arm_yaw_motor.ecd;
    arm_ctrl.yaw.enable = 0U;
    s_target_valid = 0U;
}

void arm_yaw_set_enable(uint8_t en)
{
    if (en) {
        if (servo_config_active()) {
            dbg_print("[YAW] enable rejected: servo config active\r\n");
            return;
        }
        if (arm_ctrl.teach.state == ARM_TEACH_READY ||
            arm_ctrl.teach.state == ARM_TEACH_RECORD) {
            dbg_print("[YAW] enable rejected: teaching released\r\n");
            return;
        }
        s_manual_override = 1U;
        s_motion_locked = 0U;
        s_yaw_cmd_tick = now_ms();
        s_yaw_report_force = 1U;
        if (!s_target_valid) {
            arm_ctrl.yaw.target_ecd = clamp_ecd((int32_t)arm_yaw_motor.ecd);
        }
    } else {
        PID_clear(&arm_ctrl.yaw.angle_pid);
        PID_clear(&arm_ctrl.yaw.spd_pid);
        arm_yaw_motor.tx_current = 0;
        s_target_valid = 0U;
        dji_send_current(&arm_yaw_motor, NULL, NULL, NULL);   /* 立即零输出，不等下周期 */
        s_yaw_report_force = 1U;
    }
    arm_ctrl.yaw.enable = en ? 1U : 0U;
    dbg_print("[YAW] enable=%u\r\n", (unsigned)en);
}

int arm_yaw_set_target_ecd(int32_t ecd)
{
    int32_t t = clamp_ecd(ecd);

    if (servo_config_active()) {
        dbg_print("[YAW] target rejected: servo config active\r\n");
        return -2;
    }
    if (arm_ctrl.teach.state == ARM_TEACH_READY ||
        arm_ctrl.teach.state == ARM_TEACH_RECORD) {
        dbg_print("[YAW] target rejected: teaching released\r\n");
        return -1;
    }
    arm_ctrl.yaw.target_ecd = t;
    arm_ctrl.yaw.enable = 1U;
    s_target_valid = 1U;
    s_manual_override = 1U;
    s_motion_locked = 0U;
    s_yaw_cmd_tick = now_ms();
    s_yaw_report_force = 1U;
    if (t != ecd) {
        dbg_print("[YAW] target clamped %ld -> %ld (limits %d/%d)\r\n",
                  (long)ecd, (long)t, ARM_YAW_ECD_LEFT_DEFAULT, ARM_YAW_ECD_RIGHT_DEFAULT);
    }
    return (int)t;
}

void arm_yaw_stop(void)
{
    arm_yaw_set_enable(0U);
    arm_yaw_motor.tx_current = 0;
    dji_send_current(&arm_yaw_motor, NULL, NULL, NULL);
}

static void yaw_report(uint32_t now)
{
    uint8_t need = 0U;

    if (arm_ctrl.yaw.enable) {
        if ((uint32_t)(now - s_last_report_tick) >= 50U) need = 1U;   /* 20Hz */
    } else if (s_yaw_report_force ||
               (uint32_t)(now - s_last_report_tick) >= 1000U) {
        need = 1U;
    }
    if (!need) return;

    s_yaw_report_force = 0U;
    s_last_report_tick = now;
    dbg_print("[YAW] en=%u on=%u prt=%u tgt=%ld fdb=%ld err=%ld spd=%d/%.0f out=%.0f lim=%d\r\n",
              (unsigned)arm_ctrl.yaw.enable, (unsigned)arm_ctrl.yaw.online,
              (unsigned)((ctrl_mode == PROTECT_MODE && !s_manual_override) ? 1U : 0U),
              (long)arm_ctrl.yaw.target_ecd, (long)arm_ctrl.yaw.fdb_ecd,
              (long)arm_ctrl.yaw.err_ecd,
              (int)arm_ctrl.yaw.spd_fdb, arm_ctrl.yaw.spd_ref,
              arm_ctrl.yaw.out,
              (int)arm_yaw_limit_side());
}

void arm_yaw_pid_calc(void)
{
    uint32_t now = now_ms();
    int32_t ecd;
    uint8_t enable;

    /* 反馈新鲜度：计数变化即刷新 */
    if ((int32_t)arm_yaw_motor.receive_cnt != s_last_rx_cnt) {
        s_last_rx_cnt = (int32_t)arm_yaw_motor.receive_cnt;
        s_last_rx_tick = now;
    }
    arm_ctrl.yaw.online = ((uint32_t)(now - s_last_rx_tick) < 100U) ? 1U : 0U;

    ecd = (int32_t)arm_yaw_motor.ecd;
    if (!s_target_valid) {
        arm_ctrl.yaw.target_ecd = clamp_ecd(ecd);
    }
    enable = arm_ctrl.yaw.enable;
    if (ctrl_mode == PROTECT_MODE && !s_manual_override) {
        enable = 0U;        /* 保护模式强制零输出，控制台显式会话除外 */
    }
    /* 命令保持 10s；回放期间不受此限制（512 点 * 50ms 可达 25.6s） */
    if (enable && arm_ctrl.teach.state != ARM_TEACH_PLAYBACK) {
        if ((uint32_t)(now - s_yaw_cmd_tick) > 10000U) {
            enable = 0U;
            arm_ctrl.yaw.enable = 0U;
            PID_clear(&arm_ctrl.yaw.angle_pid);
            PID_clear(&arm_ctrl.yaw.spd_pid);
            arm_yaw_motor.tx_current = 0;
            dji_send_current(&arm_yaw_motor, NULL, NULL, NULL);   /* 超时立即归零 */
            s_last_send_tick = now;
            dbg_print("[YAW] command timeout -> zero\r\n");
        }
    }
    if (arm_ctrl.teach.state == ARM_TEACH_READY ||
        arm_ctrl.teach.state == ARM_TEACH_RECORD) {
        enable = 0U;
    }

    arm_ctrl.yaw.fdb_ecd = ecd;
    arm_ctrl.yaw.err_ecd = arm_ctrl.yaw.target_ecd - ecd;
    if (arm_ctrl.teach.state == ARM_TEACH_PLAYBACK) {
        int32_t ae = (arm_ctrl.yaw.err_ecd < 0) ? -arm_ctrl.yaw.err_ecd : arm_ctrl.yaw.err_ecd;
        if (ae > arm_ctrl.teach.play_max_err) arm_ctrl.teach.play_max_err = ae;
    }

    if (!enable || !arm_ctrl.yaw.online) {
        arm_ctrl.yaw.out = 0.0f;
        arm_ctrl.yaw.spd_ref = 0.0f;
        arm_ctrl.yaw.spd_fdb = (float)arm_yaw_motor.speed_rpm;
        /* 零输出保活 50Hz：保护/超时/离线都能在 20ms 内归零 */
        if ((uint32_t)(now - s_last_send_tick) >= 20U) {
            s_last_send_tick = now;
            arm_yaw_motor.tx_current = 0;
            dji_send_current(&arm_yaw_motor, NULL, NULL, NULL);
        }
        yaw_report(now);
        return;
    }

    arm_ctrl.yaw.angle_ref = (float)arm_ctrl.yaw.target_ecd * ARM_YAW_DEG_PER_ECD;
    arm_ctrl.yaw.angle_fdb = (float)ecd * ARM_YAW_DEG_PER_ECD;

    arm_ctrl.yaw.spd_ref = pid_calc(&arm_ctrl.yaw.angle_pid,
                                    arm_ctrl.yaw.angle_fdb,
                                    arm_ctrl.yaw.angle_ref);
    arm_ctrl.yaw.spd_fdb = (float)arm_yaw_motor.speed_rpm;
    arm_ctrl.yaw.out = pid_calc(&arm_ctrl.yaw.spd_pid,
                                arm_ctrl.yaw.spd_fdb,
                                arm_ctrl.yaw.spd_ref);

    /* 边界：只允许向区间内 */
    if (ecd >= ARM_YAW_ECD_RIGHT_DEFAULT && arm_ctrl.yaw.out > 0.0f) {
        arm_ctrl.yaw.out = 0.0f;
    }
    if (ecd <= ARM_YAW_ECD_LEFT_DEFAULT && arm_ctrl.yaw.out < 0.0f) {
        arm_ctrl.yaw.out = 0.0f;
    }

    if (arm_ctrl.yaw.out > (float)arm_ctrl.yaw.out_limit) {
        arm_ctrl.yaw.out = (float)arm_ctrl.yaw.out_limit;
    } else if (arm_ctrl.yaw.out < -(float)arm_ctrl.yaw.out_limit) {
        arm_ctrl.yaw.out = -(float)arm_ctrl.yaw.out_limit;
    }

    arm_yaw_motor.tx_current = (int16_t)arm_ctrl.yaw.out;
    dji_send_current(&arm_yaw_motor, NULL, NULL, NULL);
    s_last_send_tick = now;
    yaw_report(now);
}

/* ========================= 点动 / 停止 ========================= */
int arm_joint_jog(uint8_t joint, uint16_t pulse)
{
    if (joint > 2U) return -1;
    if (servo_config_active()) return -5;
    if (arm_ctrl.teach.state == ARM_TEACH_RECORD ||
        arm_ctrl.teach.state == ARM_TEACH_PLAYBACK) return -3;
    if (arm_ctrl.teach.state == ARM_TEACH_READY) return -4;

    if (s_motion_locked) {
        s_motion_locked = 0U;
        dbg_print("[JOG] manual unlock (RC protect bypassed, 'stop' re-locks)\r\n");
    }
    s_manual_override = 1U;

    servo_set_enabled(1U);
    (void)servo_set_pulse(joint, pulse);
    s_jog_tick = now_ms();
    s_jog_active = 1U;

    dbg_print("[JOG] joint=%u pulse=%u\r\n", (unsigned)joint, (unsigned)pulse);
    return 0;
}

void arm_stop_all(void)
{
    if (arm_ctrl.teach.state != ARM_TEACH_IDLE) {
        teach_abort("stop");
    }

    arm_yaw_set_enable(0U);
    arm_yaw_motor.tx_current = 0;
    dji_send_current(&arm_yaw_motor, NULL, NULL, NULL);
    s_jog_active = 0U;
    s_motion_locked = 1U;
    s_manual_override = 0U;
    arm_busy = 0U;
    dbg_print("[ARM] stop: yaw=0, servos hold\r\n");
}

static void jog_tick(void)
{
    if (!s_jog_active) return;
    if ((uint32_t)(now_ms() - s_jog_tick) < ARM_JOG_TIMEOUT_MS) return;

    /* 关节命令 500ms 超时：伺服保持位置，yaw 由自己的 10s 保持计时管理 */
    s_jog_active = 0U;
    dbg_print("[JOG] joint command timeout (servos hold)\r\n");
}
/* ========================= 控制台入口 ========================= */
void arm_teach_cmd(const char *sub)
{
    if (sub == NULL) return;

    if (strcmp(sub, "status") == 0) {
        dbg_print("[TEACH] state=%s pts=%u origin=%d(%ld) saved=%u flash=%u seq=%lu\r\n",
                  arm_teach_state_name(), (unsigned)arm_teach_count,
                  (int)arm_ctrl.teach.origin_valid,
                  (long)arm_ctrl.teach.origin_ecd,
                  (unsigned)arm_ctrl.teach.saved_valid,
                  (unsigned)s_flash_ok, (unsigned long)s_flash_seq);
    } else if (strcmp(sub, "enter") == 0) {
        teach_enter();
    } else if (strcmp(sub, "exit") == 0) {
        teach_exit();
    } else if (strcmp(sub, "record") == 0) {
        if (arm_ctrl.teach.state == ARM_TEACH_READY) {
            arm_teach_count = 0U;
            s_last_point.yaw_ecd = (int16_t)arm_yaw_motor.ecd;
            s_last_point.shoulder = servo_get_current_pulse(0U);
            s_last_point.elbow = servo_get_current_pulse(1U);
            s_last_point.gripper = servo_get_current_pulse(2U);
            arm_ctrl.teach.last_sample_tick = now_ms();
            teach_set_state(ARM_TEACH_RECORD, "cmd");
        } else {
            dbg_print("[TEACH] record rejected: state=%s\r\n", arm_teach_state_name());
        }
    } else if (strcmp(sub, "save") == 0) {
        teach_save();
    } else if (strcmp(sub, "play") == 0) {
        teach_play_start();
    } else if (strcmp(sub, "stop") == 0) {
        if (arm_ctrl.teach.state == ARM_TEACH_PLAYBACK) {
            teach_play_stop();
        } else {
            dbg_print("[TEACH] stop ignored: state=%s\r\n", arm_teach_state_name());
        }
    } else if (strcmp(sub, "load") == 0) {
        int rc;
        if (arm_ctrl.teach.state == ARM_TEACH_READY ||
            arm_ctrl.teach.state == ARM_TEACH_RECORD ||
            arm_ctrl.teach.state == ARM_TEACH_PLAYBACK) {
            dbg_print("[TEACH] load rejected while %s\r\n", arm_teach_state_name());
            return;
        }
        rc = flash_load();
        dbg_print("[TEACH] load rc=%d pts=%u\r\n", rc, (unsigned)arm_teach_count);
        if (rc == 0) {
            teach_set_state(ARM_TEACH_SAVED, "load");
        }
    } else if (strcmp(sub, "erase") == 0) {
        teach_erase();
    } else if (strcmp(sub, "origin") == 0) {
        arm_ctrl.teach.origin_valid = 1U;
        arm_ctrl.teach.origin_ecd = (int32_t)arm_yaw_motor.ecd;
        dbg_print("[TEACH] origin ecd=%ld\r\n", (long)arm_ctrl.teach.origin_ecd);
    } else {
        dbg_print("[TEACH] usage: teach status|enter|exit|record|save|play|stop|load|erase|origin\r\n");
    }
}

/* ========================= 初始化 / 主循环 ========================= */
void arm_task_init(void)
{
    memset(&arm_ctrl, 0, sizeof(arm_ctrl));
    BSP_UserKey_Init();
    servo_init();
    yaw_pid_init();

    s_last_rx_cnt = (int32_t)arm_yaw_motor.receive_cnt;
    s_last_rx_tick = now_ms();
    s_last_send_tick = 0U;
    s_last_report_tick = 0U;
    s_yaw_report_force = 1U;
    s_jog_active = 0U;
    s_motion_locked = 1U;

    s_flash_ok = (OSPI_W25Qxx_Init() == OSPI_W25Qxx_OK) ? 1U : 0U;
    dbg_print("[ARM] init: flash=%u, limits=%d/%d, yaw out_limit=%d\r\n",
              (unsigned)s_flash_ok, ARM_YAW_ECD_LEFT_DEFAULT,
              ARM_YAW_ECD_RIGHT_DEFAULT, ARM_YAW_OUT_LIMIT);

    if (s_flash_ok && flash_load() == 0) {
        dbg_print("[ARM] flash track loaded: pts=%u seq=%lu (state=IDLE, double=SAVED)\r\n",
                  (unsigned)arm_teach_count, (unsigned long)s_flash_seq);
    }

    vision_uart_init();        /* UART8/PE0 视觉 RX，PC USB-TTL 可直接模拟 */

    arm_yaw_motor.tx_current = 0;
    dji_send_current(&arm_yaw_motor, NULL, NULL, NULL);
}

void arm_task(void const *argu)
{
    (void)argu;
    arm_task_init();

    for (;;) {
        key_event_e ev;
        servo_poll();          /* 总线舵机解析 + 控制台一行 */
        servo_test_poll();     /* yaw monitor（旧命令） */
        ev = BSP_UserKey_Poll();
        if (ev != KEY_EVT_NONE) handle_key(ev);
        teach_tick();
        arm_yaw_pid_calc();
        jog_tick();
        osDelay(ARM_TASK_PERIOD);
    }
}

/* prot_vision.c 的弱回调占位 */
void arm_vision_callback(void)
{
}
