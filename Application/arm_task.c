/**
  * @file    arm_task.c
  * @brief   机械臂：yaw 双环 + 单按钮拖动示教 + 四关节点动
  * @note    CAN 发送仍走 dji_send_current -> CAN2 队列；轨迹存片内 Flash 最后两个扇区。
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
#include "dbg_print.h"
#include "cmsis_os.h"
#include "pid.h"
#include "stm32h7xx_hal_flash.h"
#include "stm32h7xx_hal_flash_ex.h"
#include <string.h>
#include <stdlib.h>

/* ========================= 常量 ========================= */
#define ARM_YAW_DEG_PER_ECD     (360.0f / 8192.0f)
#define PID_ERR_LAST            1       /* pid_t.err[] 的 LAST 下标（见 Math/pid.c） */

/* 片内 Flash：单 bank 1MB、8 个 128KB 扇区，用最后两个。固件约 134KB，碰不到这里。 */
#define TEACH_FLASH_A_ADDR      0x080C0000UL
#define TEACH_FLASH_B_ADDR      0x080E0000UL
#define TEACH_FLASH_A_SECTOR    FLASH_SECTOR_6
#define TEACH_FLASH_B_SECTOR    FLASH_SECTOR_7
#define TEACH_MAGIC             0x41544C32UL   /* "ATL2" */
#define TEACH_VER               2U
#define TEACH_HDR_SIZE          16U

#define TEACH_TICK_MS           ARM_TEACH_SAMPLE_MS
#define TEACH_DYAW_ECD          4
#define TEACH_DSH_US            5
#define TEACH_DEL_US            5
#define TEACH_DGR_US            10
#define TEACH_PAUSE_TICKS       4U      /* >=4 个 tick 没有新点视为暂停：补“保持点” */
#define TEACH_PLAY_GO_MS        800U    /* 回放前先用这么长时间回到起始姿态 */
#define TEACH_PLAY_ARRIVE_MS    1500U
#define TEACH_PLAY_ARRIVE_ECD   30

#define ARM_YAW_RUNAWAY_ERR_ECD 30
#define ARM_YAW_RUNAWAY_SPD_RPM 50.0f
#define ARM_YAW_RUNAWAY_MS      80U
#define ARM_YAW_PROBE_MS        150U
#define ARM_YAW_PROBE_OUT       4000

/* 关节机械行程没在这台臂上验证过，默认给窄范围，joint limit 放宽 */
#define ARM_JOINT_DFLT_MIN      1400U   /* 肩/肘：中心 ±100us */
#define ARM_JOINT_DFLT_MAX      1600U
#define ARM_GRIPPER_DFLT_MIN    1250U   /* 夹爪：旧固件实际到过的范围约 1259~2062us */
#define ARM_GRIPPER_DFLT_MAX    2060U

/* ========================= 全局 ========================= */
arm_ctrl_t arm_ctrl;
arm_teach_point_t arm_teach_points[ARM_TEACH_MAX_POINTS];
uint16_t arm_teach_count;
volatile uint8_t arm_busy;

static uint8_t  s_flash_ok;
static uint32_t s_flash_seq;

static arm_teach_point_t s_last_point;
static uint16_t s_sample_gap;        /* 距上一个已存点的采样 tick 数 */

static uint16_t s_joint_min[3];
static uint16_t s_joint_max[3];

static int32_t  s_last_rx_cnt;
static uint32_t s_last_rx_tick;
static uint32_t s_last_send_tick;
static uint32_t s_last_report_tick;
static uint32_t s_last_calc_tick;
static uint32_t s_yaw_cmd_tick;
static uint32_t s_runaway_tick;
static uint8_t  s_target_valid;      /* 0=目标跟随当前 ECD，避免首次使能猛冲 */
static uint8_t  s_yaw_report_force;

static uint8_t  s_play_phase;        /* 0=回起始姿态, 1=逐段插值 */
static uint16_t s_play_idx;          /* 当前段的起点下标 */
static uint32_t s_play_t_ms;         /* 当前段内已进行时间 */
static uint32_t s_play_tick;
static uint32_t s_play_phase_tick;

/* Flash 镜像暂存：头 16B + 512 点 * 10B，补齐到 32B 的整数倍（H7 一个编程字 = 32B） */
#define TEACH_IMG_BYTES     (TEACH_HDR_SIZE + (uint32_t)ARM_TEACH_MAX_POINTS * sizeof(arm_teach_point_t))
#define TEACH_IMG_WORDS     (((TEACH_IMG_BYTES + 31U) / 32U) * 8U)
static __align(32) uint32_t s_flash_img[TEACH_IMG_WORDS];

static void teach_set_state(arm_teach_state_e s, const char *why);
static void teach_exit(const char *why);
static void teach_abort(const char *why);
static void teach_play_stop(const char *why);
static void play_stop_servos(void);

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

static int32_t iabs32(int32_t v)
{
    return (v < 0) ? -v : v;
}

static int32_t lerp_i32(int32_t a, int32_t b, uint32_t t, uint32_t span)
{
    if (span == 0U) return b;
    return a + (int32_t)((float)(b - a) * (float)t / (float)span);
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

static const char *state_name(arm_teach_state_e s)
{
    switch (s) {
        case ARM_TEACH_IDLE:     return "IDLE";
        case ARM_TEACH_READY:    return "READY";
        case ARM_TEACH_RECORD:   return "RECORD";
        case ARM_TEACH_SAVED:    return "SAVED";
        case ARM_TEACH_PLAYBACK: return "PLAYBACK";
        case ARM_TEACH_ABORT:    return "ABORT";
        default:                 return "?";
    }
}

const char *arm_teach_state_name(void)
{
    return state_name(arm_ctrl.teach.state);
}

uint8_t arm_teach_is_recording(void)
{
    return (arm_ctrl.teach.state == ARM_TEACH_RECORD) ? 1U : 0U;
}

static const char *joint_name(uint8_t joint)
{
    return (joint == 0U) ? "shoulder" : (joint == 1U) ? "elbow" : "gripper";
}

static uint16_t joint_clamp(uint8_t joint, uint16_t pulse)
{
    if (pulse < s_joint_min[joint]) pulse = s_joint_min[joint];
    if (pulse > s_joint_max[joint]) pulse = s_joint_max[joint];
    return pulse;
}

/* ========================= 模式门控 ========================= */
uint8_t arm_motion_allowed(void)
{
    return (ctrl_mode == REMOTER_MODE || ctrl_mode == ARM_TEACH_MODE) ? 1U : 0U;
}

const char *arm_mode_name(void)
{
    switch (ctrl_mode) {
        case PROTECT_MODE:   return "PROTECT";
        case REMOTER_MODE:   return "REMOTER";
        case KEYBOARD_MODE:  return "KEYBOARD";
        case VISION_MODE:    return "VISION";
        case ARM_TEACH_MODE: return "ARM_TEACH";
        default:             return "?";
    }
}

int arm_mode_manual(void)
{
    arm_teach_state_e st = arm_ctrl.teach.state;
    if (st == ARM_TEACH_READY || st == ARM_TEACH_RECORD || st == ARM_TEACH_PLAYBACK) {
        dbg_print("[MODE] manual rejected while teach=%s\r\n", state_name(st));
        return -1;
    }
    mode_user_set(REMOTER_MODE);
    dbg_print("[MODE] REMOTER (console manual): joint/yaw accepted, 'mode protect' to leave\r\n");
    return 0;
}

/* ========================= 片内 Flash 轨迹 ========================= */
typedef struct {
    uint32_t magic;
    uint32_t seq;
    uint32_t crc;
    uint16_t count;
    uint16_t ver;
} teach_hdr_t;

static uint32_t flash_build_image(void)
{
    teach_hdr_t *hdr = (teach_hdr_t *)s_flash_img;
    uint32_t n = (uint32_t)arm_teach_count * sizeof(arm_teach_point_t);
    uint32_t total = (TEACH_HDR_SIZE + n + 31U) & ~31U;

    memset(s_flash_img, 0xFF, total);
    hdr->magic = TEACH_MAGIC;
    hdr->seq   = s_flash_seq;
    hdr->count = arm_teach_count;
    hdr->ver   = TEACH_VER;
    hdr->crc   = crc32_calc((const uint8_t *)arm_teach_points, n);
    memcpy((uint8_t *)s_flash_img + TEACH_HDR_SIZE, arm_teach_points, n);
    return total;
}

/* 擦除期间 CPU 从同一 bank 取指会被停住（未实测，估计 1~2 秒），调用前必须保证没有运动在发 */
static int flash_erase_sector(uint32_t sector)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t sector_err = 0U;

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG_BANK1(FLASH_FLAG_ALL_ERRORS_BANK1);
    erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase.Banks        = FLASH_BANK_1;
    erase.Sector       = sector;
    erase.NbSectors    = 1U;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_4;
    if (HAL_FLASHEx_Erase(&erase, &sector_err) != HAL_OK) {
        HAL_FLASH_Lock();
        dbg_print("[TEACH] flash erase failed: sector=%lu err=%lu\r\n",
                  (unsigned long)sector, (unsigned long)sector_err);
        return -1;
    }
    return 0;
}

static uint8_t flash_slot_blank(uint32_t addr)
{
    return (*(const uint32_t *)(uintptr_t)addr == 0xFFFFFFFFUL) ? 1U : 0U;
}

/* 写之前冻结底盘并让零值帧发出去；返回原来的 arm_busy */
static uint8_t flash_guard_enter(void)
{
    uint8_t prev = arm_busy;
    arm_busy = 1U;
    arm_yaw_motor.tx_current = 0;
    dji_send_current(&arm_yaw_motor, NULL, NULL, NULL);
    osDelay(40);
    return prev;
}

static void flash_guard_exit(uint8_t prev)
{
    arm_busy = prev;
}

static int flash_save(void)
{
    uint32_t addr, sector, total, i, t0;
    int rc = 0;

    if (!s_flash_ok || arm_teach_count < 2U) return -1;

    s_flash_seq++;
    total = flash_build_image();
    if ((s_flash_seq & 1U) != 0U) {
        addr = TEACH_FLASH_B_ADDR;
        sector = TEACH_FLASH_B_SECTOR;
    } else {
        addr = TEACH_FLASH_A_ADDR;
        sector = TEACH_FLASH_A_SECTOR;
    }

    dbg_print("[TEACH] saving %u pts to slot %s, flash busy (CPU stalls, keep still)...\r\n",
              (unsigned)arm_teach_count, (addr == TEACH_FLASH_B_ADDR) ? "B" : "A");
    t0 = now_ms();

    if (flash_erase_sector(sector) != 0) return -2;
    __HAL_FLASH_CLEAR_FLAG_BANK1(FLASH_FLAG_ALL_ERRORS_BANK1);
    for (i = 0U; i < total; i += 32U) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, addr + i,
                              (uint32_t)(uintptr_t)&s_flash_img[i / 4U]) != HAL_OK) {
            rc = -3;
            break;
        }
    }
    HAL_FLASH_Lock();
    if (rc == 0 && memcmp((const void *)(uintptr_t)addr, s_flash_img, total) != 0) rc = -4;

    dbg_print("[TEACH] flash %s: seq=%lu, %lums\r\n",
              (rc == 0) ? "ok" : "FAILED", (unsigned long)s_flash_seq,
              (unsigned long)(now_ms() - t0));
    return rc;
}

static int flash_load(void)
{
    uint8_t i, found = 0U;
    int32_t best = 0;

    if (!s_flash_ok) return -1;

    for (i = 0U; i < 2U; i++) {
        const teach_hdr_t *h = (const teach_hdr_t *)(uintptr_t)((i == 0U) ? TEACH_FLASH_A_ADDR : TEACH_FLASH_B_ADDR);
        const arm_teach_point_t *pts = (const arm_teach_point_t *)((uintptr_t)h + TEACH_HDR_SIZE);

        if (h->magic != TEACH_MAGIC || h->ver != TEACH_VER) continue;
        if (h->count < 2U || h->count > ARM_TEACH_MAX_POINTS) continue;
        if (crc32_calc((const uint8_t *)pts, (uint32_t)h->count * sizeof(arm_teach_point_t)) != h->crc) {
            dbg_print("[TEACH] slot %u crc error, skipped\r\n", (unsigned)i);
            continue;
        }
        if (found == 0U || (int32_t)(h->seq - best) > 0) {
            memcpy(arm_teach_points, pts, (uint32_t)h->count * sizeof(arm_teach_point_t));
            arm_teach_count = h->count;
            best = (int32_t)h->seq;
            found = 1U;
        }
    }

    if (found != 0U) {
        s_flash_seq = (uint32_t)best;
        arm_ctrl.teach.saved_valid = 1U;
        return 0;
    }
    return -2;
}

/* ========================= 示教状态 ========================= */
static void teach_set_state(arm_teach_state_e s, const char *why)
{
    if (arm_ctrl.teach.state != s) {
        dbg_print("[TEACH] %s -> %s (%s) pts=%u\r\n",
                  state_name(arm_ctrl.teach.state), state_name(s),
                  (why != NULL) ? why : "", (unsigned)arm_teach_count);
    }
    arm_ctrl.teach.state = s;
    arm_ctrl.teach.state_tick = now_ms();
    /* 只在释力和回放期间锁底盘；SAVED、ABORT、IDLE 不影响底盘 */
    arm_busy = (s == ARM_TEACH_READY || s == ARM_TEACH_RECORD ||
                s == ARM_TEACH_PLAYBACK) ? 1U : 0U;
}

static int teach_read_joints(arm_teach_point_t *p)
{
    uint8_t attempt;

    for (attempt = 0U; attempt < 3U; attempt++) {
        if (xb_servo_read_pos(0U, &p->shoulder) == 0U &&
            xb_servo_read_pos(1U, &p->elbow) == 0U) {
            p->gripper = servo_get_current_pulse(2U);
            p->yaw_ecd = (int16_t)arm_yaw_motor.ecd;
            p->dt50 = 0U;
            p->reserved = 0U;
            return 0;
        }
    }
    return -1;
}

static uint8_t teach_moved(const arm_teach_point_t *p)
{
    int32_t dy = iabs32((int32_t)p->yaw_ecd - (int32_t)s_last_point.yaw_ecd);
    int32_t ds = iabs32((int32_t)p->shoulder - (int32_t)s_last_point.shoulder);
    int32_t de = iabs32((int32_t)p->elbow - (int32_t)s_last_point.elbow);
    int32_t dg = iabs32((int32_t)p->gripper - (int32_t)s_last_point.gripper);

    return (dy >= TEACH_DYAW_ECD || ds >= TEACH_DSH_US ||
            de >= TEACH_DEL_US || dg >= TEACH_DGR_US) ? 1U : 0U;
}

/* 存一个点。gap = 距上一个已存点的 tick 数，首点传 0 */
static uint8_t teach_push(const arm_teach_point_t *p, uint16_t gap)
{
    arm_teach_point_t q = *p;

    while (gap > 255U) {               /* 超长间隔拆成多个 255 tick 的保持点 */
        if (arm_teach_count >= ARM_TEACH_MAX_POINTS) return 0U;
        s_last_point.dt50 = 255U;
        arm_teach_points[arm_teach_count++] = s_last_point;
        gap -= 255U;
    }
    if (arm_teach_count >= ARM_TEACH_MAX_POINTS) return 0U;
    q.dt50 = (uint8_t)gap;
    arm_teach_points[arm_teach_count++] = q;
    s_last_point = q;
    return 1U;
}

static void teach_enter(void)
{
    arm_teach_state_e st = arm_ctrl.teach.state;

    if (st == ARM_TEACH_READY || st == ARM_TEACH_RECORD || st == ARM_TEACH_PLAYBACK) return;
    if (servo_config_active()) {
        dbg_print("[TEACH] enter rejected: servo config active ('servo run' first)\r\n");
        return;
    }

    mode_user_set(ARM_TEACH_MODE);
    arm_yaw_set_enable(0U);            /* 先断 yaw 转矩，再给肩/肘释力 */
    servo_set_enabled(1U);
    if (servo_teach_begin() != 0U) {
        teach_abort("servo release failed");
        return;
    }

    arm_teach_count = 0U;
    memset(&s_last_point, 0, sizeof(s_last_point));
    s_sample_gap = 0U;
    arm_ctrl.teach.count = 0U;
    arm_ctrl.teach.origin_valid = 0U;
    arm_ctrl.teach.origin_ecd = 0;
    arm_ctrl.teach.saved_valid = 0U;
    arm_ctrl.teach.play_index = 0U;
    arm_ctrl.teach.delete_confirm = 0U;
    teach_set_state(ARM_TEACH_READY, "enter");
    dbg_print("[TEACH] released. short=origin/start/key point, double=save/play, long5=erase, long2=exit\r\n");
}

static void teach_exit(const char *why)
{
    arm_teach_state_e prev = arm_ctrl.teach.state;

    servo_teach_end();
    servo_set_enabled(1U);
    if (prev == ARM_TEACH_PLAYBACK) play_stop_servos();
    arm_yaw_set_enable(0U);
    if (prev == ARM_TEACH_READY || prev == ARM_TEACH_RECORD) {
        arm_ctrl.teach.saved_valid = 0U;    /* 没保存的录制不能回放 */
    }
    mode_user_clear();
    teach_set_state(ARM_TEACH_IDLE, why);
}

static void teach_abort(const char *why)
{
    arm_teach_state_e prev = arm_ctrl.teach.state;

    servo_teach_end();
    servo_set_enabled(1U);
    if (prev == ARM_TEACH_PLAYBACK) play_stop_servos();
    arm_yaw_set_enable(0U);
    if (prev == ARM_TEACH_READY || prev == ARM_TEACH_RECORD) {
        arm_ctrl.teach.saved_valid = 0U;    /* 录制中途出问题：这段 RAM 数据不能回放 */
    }
    mode_user_clear();
    teach_set_state(ARM_TEACH_ABORT, why);
}

/* ========================= 录制 ========================= */
static void teach_record_start(void)
{
    arm_teach_point_t p;

    if (teach_read_joints(&p) != 0) {
        dbg_print("[TEACH] cannot read servos, stay READY\r\n");
        return;
    }
    arm_teach_count = 0U;
    memset(&s_last_point, 0, sizeof(s_last_point));
    s_sample_gap = 0U;
    (void)teach_push(&p, 0U);          /* 起始姿态作为第 0 点 */
    arm_ctrl.teach.last_sample_tick = now_ms();
    teach_set_state(ARM_TEACH_RECORD, "record");
    dbg_print("[TEACH] pt0 yaw=%d sh=%u el=%u gr=%u\r\n", (int)p.yaw_ecd,
              (unsigned)p.shoulder, (unsigned)p.elbow, (unsigned)p.gripper);
}

static void teach_sample(void)
{
    arm_teach_point_t p;
    uint16_t gap;

    if (arm_teach_count >= ARM_TEACH_MAX_POINTS) {
        teach_abort("track full");
        return;
    }
    if (teach_read_joints(&p) != 0) {
        teach_abort("servo offline");
        return;
    }
    if (s_sample_gap < 0xFFFFU) s_sample_gap++;
    if (teach_moved(&p) == 0U) return;

    gap = s_sample_gap;
    if (gap >= TEACH_PAUSE_TICKS) {
        /* 停了一阵再动：用保持点占住暂停时间，再用 1 tick 走到新点 */
        if (teach_push(&s_last_point, (uint16_t)(gap - 1U)) == 0U) {
            teach_abort("track full");
            return;
        }
        gap = 1U;
    }
    if (teach_push(&p, gap) == 0U) {
        teach_abort("track full");
        return;
    }
    s_sample_gap = 0U;
}

static void teach_mark_key(void)
{
    arm_teach_point_t p;
    uint16_t gap = s_sample_gap;

    if (teach_read_joints(&p) != 0) {
        dbg_print("[TEACH] key point skipped: servo read failed\r\n");
        return;
    }
    if (gap == 0U) gap = 1U;
    if (teach_push(&p, gap) == 0U) {
        teach_abort("track full");
        return;
    }
    s_sample_gap = 0U;
    dbg_print("[TEACH] key pt=%u yaw=%d sh=%u el=%u gr=%u\r\n", (unsigned)arm_teach_count,
              (int)p.yaw_ecd, (unsigned)p.shoulder, (unsigned)p.elbow, (unsigned)p.gripper);
}

static void teach_save(void)
{
    arm_teach_point_t p;
    uint16_t gap;
    int rc;

    if (arm_ctrl.teach.state != ARM_TEACH_RECORD) {
        dbg_print("[TEACH] save rejected: state=%s\r\n", arm_teach_state_name());
        return;
    }
    if (teach_read_joints(&p) == 0 && teach_moved(&p) != 0U) {
        gap = s_sample_gap;
        if (gap == 0U) gap = 1U;
        if (gap >= TEACH_PAUSE_TICKS) {
            (void)teach_push(&s_last_point, (uint16_t)(gap - 1U));
            gap = 1U;
        }
        (void)teach_push(&p, gap);     /* 终点 */
    }
    if (arm_teach_count < 2U) {
        dbg_print("[TEACH] save rejected: only %u pt, move the arm first (long2=exit)\r\n",
                  (unsigned)arm_teach_count);
        return;
    }

    servo_teach_end();                 /* 锁住肩/肘，防止保存后下落 */
    servo_set_enabled(1U);
    arm_yaw_set_enable(0U);
    rc = flash_save();
    if (rc != 0) {
        dbg_print("[TEACH] flash save failed rc=%d, RAM copy kept (lost on power-off)\r\n", rc);
    }
    arm_ctrl.teach.saved_valid = 1U;
    teach_set_state(ARM_TEACH_SAVED, (rc == 0) ? "saved" : "saved RAM only");
}

static void teach_erase(void)
{
    arm_teach_state_e st = arm_ctrl.teach.state;
    uint8_t busy_prev;

    if (st == ARM_TEACH_READY || st == ARM_TEACH_RECORD || st == ARM_TEACH_PLAYBACK) {
        dbg_print("[TEACH] erase rejected while %s\r\n", state_name(st));
        return;
    }
    if (st != ARM_TEACH_IDLE) teach_exit("erase");

    dbg_print("[TEACH] erasing flash slots (CPU stalls, keep still)...\r\n");
    busy_prev = flash_guard_enter();
    if (s_flash_ok != 0U) {
        if (flash_slot_blank(TEACH_FLASH_A_ADDR) == 0U) (void)flash_erase_sector(TEACH_FLASH_A_SECTOR);
        if (flash_slot_blank(TEACH_FLASH_B_ADDR) == 0U) (void)flash_erase_sector(TEACH_FLASH_B_SECTOR);
        HAL_FLASH_Lock();
    }
    s_flash_seq = 0U;

    arm_teach_count = 0U;
    arm_ctrl.teach.count = 0U;
    arm_ctrl.teach.saved_valid = 0U;
    arm_ctrl.teach.origin_valid = 0U;
    arm_ctrl.teach.origin_ecd = 0;
    arm_ctrl.teach.delete_confirm = 0U;
    flash_guard_exit(busy_prev);
    dbg_print("[TEACH] track erased (RAM+flash)\r\n");
}

/* 只用 USB 供电也能做的 Flash 验证：走真实的保存/加载路径写 4 个合成点，回读比对，再擦掉。
 * 写的是“较旧”的那个槽，当前有效的轨迹槽不动。 */
void arm_flash_test(void)
{
    arm_teach_state_e st = arm_ctrl.teach.state;
    uint8_t busy_prev, keep_saved = arm_ctrl.teach.saved_valid;
    uint32_t keep_seq = s_flash_seq;
    uint32_t sector;
    int rc, loaded;
    uint8_t i, ok;

    if (st == ARM_TEACH_READY || st == ARM_TEACH_RECORD || st == ARM_TEACH_PLAYBACK) {
        dbg_print("[TEACH] flash test rejected while %s\r\n", state_name(st));
        return;
    }
    if (s_flash_ok == 0U) {
        dbg_print("[TEACH] flash test rejected: image layout invalid\r\n");
        return;
    }

    dbg_print("[TEACH] flash test: 4 synthetic pts, write+verify+erase (CPU stalls, keep still)\r\n");
    busy_prev = flash_guard_enter();

    for (i = 0U; i < 4U; i++) {
        memset(&arm_teach_points[i], 0, sizeof(arm_teach_point_t));
        arm_teach_points[i].yaw_ecd = (int16_t)(2000 + 100 * (int)i);
        arm_teach_points[i].shoulder = (uint16_t)(1400 + 20 * i);
        arm_teach_points[i].elbow = (uint16_t)(1600 - 20 * i);
        arm_teach_points[i].gripper = 1500U;
        arm_teach_points[i].dt50 = (uint8_t)(2U + i);
    }
    arm_teach_count = 4U;

    rc = flash_save();                         /* seq+1，写到另一个槽 */

    memset(arm_teach_points, 0, 4U * sizeof(arm_teach_point_t));
    arm_teach_count = 0U;
    arm_ctrl.teach.saved_valid = 0U;
    s_flash_seq = keep_seq;
    loaded = flash_load();                     /* 应该读到刚写的 4 个点 */
    ok = (loaded == 0 && arm_teach_count == 4U &&
          arm_teach_points[0].yaw_ecd == 2000 &&
          arm_teach_points[3].yaw_ecd == 2300 &&
          arm_teach_points[3].dt50 == 5U) ? 1U : 0U;

    /* 擦掉测试写入的槽（按 keep_seq+1 的奇偶算，不能依赖 load 的结果，防止误擦有效槽） */
    if (((keep_seq + 1U) & 1U) != 0U) {
        sector = TEACH_FLASH_B_SECTOR;
    } else {
        sector = TEACH_FLASH_A_SECTOR;
    }
    if (flash_erase_sector(sector) != 0) ok = 0U;
    HAL_FLASH_Lock();
    s_flash_seq = keep_seq;

    arm_teach_count = 0U;
    arm_ctrl.teach.saved_valid = 0U;
    if (keep_saved != 0U) {
        if (flash_load() != 0) dbg_print("[TEACH] WARNING: previous RAM-only track lost by flash test\r\n");
    }
    flash_guard_exit(busy_prev);
    dbg_print("[TEACH] flash test %s (save rc=%d, load=%d)\r\n", (ok != 0U) ? "PASS" : "FAIL", rc, loaded);
}

/* ========================= 回放 ========================= */
static uint32_t seg_ms_of(uint16_t idx)
{
    uint32_t ms = (uint32_t)arm_teach_points[idx].dt50 * TEACH_TICK_MS;
    return (ms == 0U) ? TEACH_TICK_MS : ms;
}

/* 总线舵机用 seg_ms 时间走到第 idx 点：舵机内部做线性插值 */
static void play_servo_to(uint16_t idx, uint32_t seg_ms)
{
    const arm_teach_point_t *p = &arm_teach_points[idx];

    if (seg_ms < 20U) seg_ms = 20U;
    if (seg_ms > XB_TIME_MAX_MS) seg_ms = XB_TIME_MAX_MS;
    servo_set_pulse_time(0U, joint_clamp(0U, p->shoulder), (uint16_t)seg_ms);
    servo_set_pulse_time(1U, joint_clamp(1U, p->elbow), (uint16_t)seg_ms);
}

/* 停在当前位置：读回实际位置并保持 */
static void play_stop_servos(void)
{
    uint16_t pulse;
    uint8_t j;

    for (j = 0U; j < XB_JOINT_NUM; j++) {
        if (xb_servo_read_pos(j, &pulse) == 0U) {
            if (xb_servo_set_pos(j, pulse, 20U) == 0U) servo_param[j].target_pulse = pulse;
        }
    }
}

static void teach_play_start(void)
{
    const arm_teach_point_t *p;

    if (arm_ctrl.teach.state == ARM_TEACH_PLAYBACK) return;
    if (arm_teach_count < 2U || arm_ctrl.teach.saved_valid == 0U) {
        dbg_print("[TEACH] play rejected: no saved track\r\n");
        return;
    }
    if (servo_config_active()) {
        dbg_print("[TEACH] play rejected: servo config active\r\n");
        return;
    }
    if (servo_online(0U) == 0U || servo_online(1U) == 0U) {
        dbg_print("[TEACH] play rejected: shoulder/elbow offline\r\n");
        return;
    }
    if (arm_ctrl.yaw.online == 0U) {
        dbg_print("[TEACH] play rejected: yaw feedback lost\r\n");
        return;
    }
    if (arm_ctrl.yaw.dir_valid == 0U) {
        dbg_print("[TEACH] play rejected: yaw direction not calibrated, send 'yaw probe' first\r\n");
        return;
    }

    servo_set_enabled(1U);
    servo_teach_end();
    mode_user_set(ARM_TEACH_MODE);

    s_play_phase = 0U;
    s_play_idx = 0U;
    s_play_t_ms = 0U;
    s_play_phase_tick = now_ms();
    s_play_tick = s_play_phase_tick;
    arm_ctrl.teach.play_index = 0U;
    arm_ctrl.teach.play_max_err = 0;

    p = &arm_teach_points[0];
    play_servo_to(0U, TEACH_PLAY_GO_MS);
    (void)servo_set_pulse(2U, joint_clamp(2U, p->gripper));
    teach_set_state(ARM_TEACH_PLAYBACK, "start");

    arm_ctrl.yaw.target_ecd = clamp_ecd((int32_t)p->yaw_ecd);
    arm_ctrl.yaw.enable = 1U;
    s_target_valid = 1U;
    s_runaway_tick = 0U;
    s_last_calc_tick = now_ms();
    dbg_print("[TEACH] playback: back to pt0 first, then %u pts\r\n", (unsigned)arm_teach_count);
}

static void teach_play_stop(const char *why)
{
    if (arm_ctrl.teach.state != ARM_TEACH_PLAYBACK) return;

    arm_ctrl.yaw.enable = 0U;
    PID_clear(&arm_ctrl.yaw.angle_pid);
    PID_clear(&arm_ctrl.yaw.spd_pid);
    arm_yaw_motor.tx_current = 0;
    dji_send_current(&arm_yaw_motor, NULL, NULL, NULL);
    s_last_send_tick = now_ms();
    play_stop_servos();
    s_target_valid = 0U;
    dbg_print("[TEACH] playback %s: idx=%u/%u, max_yaw_err=%ld ECD\r\n",
              why, (unsigned)s_play_idx, (unsigned)arm_teach_count,
              (long)arm_ctrl.teach.play_max_err);
    teach_set_state(ARM_TEACH_SAVED, why);
}

static void teach_play_tick(void)
{
    uint32_t now = now_ms(), seg;
    const arm_teach_point_t *a, *b;

    if (s_play_phase == 0U) {
        /* 等回到起始姿态附近，或超时 */
        if ((uint32_t)(now - s_play_phase_tick) >= TEACH_PLAY_ARRIVE_MS ||
            iabs32(arm_ctrl.yaw.err_ecd) <= TEACH_PLAY_ARRIVE_ECD) {
            s_play_phase = 1U;
            s_play_idx = 0U;
            s_play_t_ms = 0U;
            s_play_tick = now;
            play_servo_to(1U, seg_ms_of(1U));
        }
        return;
    }

    if ((uint32_t)(now - s_play_tick) < TEACH_TICK_MS) return;
    s_play_tick += TEACH_TICK_MS;
    if ((uint32_t)(now - s_play_tick) >= TEACH_TICK_MS) s_play_tick = now;   /* 落后太多则重新对齐 */

    for (;;) {
        seg = seg_ms_of((uint16_t)(s_play_idx + 1U));
        if (s_play_t_ms + TEACH_TICK_MS < seg) {
            s_play_t_ms += TEACH_TICK_MS;
            break;
        }
        s_play_idx++;                  /* 本段走完，进入下一段 */
        s_play_t_ms = 0U;
        if ((uint16_t)(s_play_idx + 1U) >= arm_teach_count) {
            teach_play_stop("done");
            return;
        }
        play_servo_to((uint16_t)(s_play_idx + 1U), seg_ms_of((uint16_t)(s_play_idx + 1U)));
    }
    arm_ctrl.teach.play_index = s_play_idx;

    /* yaw 和夹爪按段内时间线性插值；肩/肘由舵机自己插值 */
    a = &arm_teach_points[s_play_idx];
    b = &arm_teach_points[s_play_idx + 1U];
    arm_ctrl.yaw.target_ecd = clamp_ecd(lerp_i32((int32_t)a->yaw_ecd, (int32_t)b->yaw_ecd, s_play_t_ms, seg));
    (void)servo_set_pulse(2U, joint_clamp(2U, (uint16_t)lerp_i32((int32_t)a->gripper, (int32_t)b->gripper, s_play_t_ms, seg)));
}

static void teach_tick(void)
{
    uint32_t now = now_ms();

    arm_ctrl.teach.count = arm_teach_count;

    if (arm_ctrl.teach.delete_confirm != 0U &&
        (int32_t)(now - arm_ctrl.teach.delete_deadline) >= 0) {
        arm_ctrl.teach.delete_confirm = 0U;
        dbg_print("[TEACH] delete confirm timeout\r\n");
    }

    if (arm_ctrl.teach.state == ARM_TEACH_RECORD) {
        if ((uint32_t)(now - arm_ctrl.teach.last_sample_tick) >= TEACH_TICK_MS) {
            arm_ctrl.teach.last_sample_tick += TEACH_TICK_MS;
            if ((uint32_t)(now - arm_ctrl.teach.last_sample_tick) >= TEACH_TICK_MS) {
                arm_ctrl.teach.last_sample_tick = now;
            }
            teach_sample();
        }
    } else if (arm_ctrl.teach.state == ARM_TEACH_PLAYBACK) {
        teach_play_tick();
    }
}

/* ========================= 按键 ========================= */
static void on_short(void)
{
    switch (arm_ctrl.teach.state) {
        case ARM_TEACH_READY:
            if (arm_ctrl.teach.origin_valid == 0U) {
                arm_ctrl.teach.origin_valid = 1U;
                arm_ctrl.teach.origin_ecd = (int32_t)arm_yaw_motor.ecd;
                dbg_print("[TEACH] origin ecd=%ld, short again to start record\r\n",
                          (long)arm_ctrl.teach.origin_ecd);
            } else {
                teach_record_start();
            }
            break;
        case ARM_TEACH_RECORD:
            teach_mark_key();
            break;
        case ARM_TEACH_PLAYBACK:
            teach_play_stop("short");
            break;
        case ARM_TEACH_SAVED:
            dbg_print("[TEACH] SAVED: double=play, long2=exit, long5 x2=erase\r\n");
            break;
        case ARM_TEACH_ABORT:
            dbg_print("[TEACH] ABORT: long2 to clear\r\n");
            break;
        default:
            break;
    }
}

static void on_double(void)
{
    int rc;

    switch (arm_ctrl.teach.state) {
        case ARM_TEACH_RECORD:
            teach_save();
            break;
        case ARM_TEACH_SAVED:
            teach_play_start();
            break;
        case ARM_TEACH_PLAYBACK:
            teach_play_stop("double");
            break;
        case ARM_TEACH_IDLE:
        case ARM_TEACH_ABORT:
            /* 只回放已保存的轨迹；录制中断留下的半截 RAM 数据不能回放 */
            if (arm_ctrl.teach.saved_valid == 0U) {
                rc = flash_load();
                dbg_print("[TEACH] load flash rc=%d pts=%u\r\n", rc, (unsigned)arm_teach_count);
                if (rc != 0) return;
            }
            if (arm_teach_count >= 2U) teach_set_state(ARM_TEACH_SAVED, "load");
            break;
        default:
            dbg_print("[TEACH] double ignored in %s\r\n", arm_teach_state_name());
            break;
    }
}

static void on_long2(void)
{
    if (arm_ctrl.teach.state == ARM_TEACH_IDLE) {
        teach_enter();
    } else {
        teach_exit("long2");
    }
}

static void on_long5(void)
{
    if (arm_ctrl.teach.state != ARM_TEACH_SAVED) {
        dbg_print("[TEACH] long5 ignored in %s\r\n", arm_teach_state_name());
        return;
    }
    if (arm_ctrl.teach.delete_confirm == 0U) {
        arm_ctrl.teach.delete_confirm = 1U;
        arm_ctrl.teach.delete_deadline = now_ms() + ARM_TEACH_DELETE_CONFIRM_MS;
        dbg_print("[TEACH] erase saved track? hold 5s again within %us\r\n",
                  (unsigned)(ARM_TEACH_DELETE_CONFIRM_MS / 1000U));
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
/* yaw 命令的准入检查：返回 0 才允许非零输出，拒绝原因已打印。
 * need_dir=0 用于 yaw probe：它本身就是用来标定方向的。 */
static int yaw_gate(const char *cmd, uint8_t need_dir)
{
    arm_teach_state_e st = arm_ctrl.teach.state;

    if (servo_config_active()) {
        dbg_print("[YAW] %s rejected: servo config active ('servo run' first)\r\n", cmd);
        return -1;
    }
    if (st == ARM_TEACH_READY || st == ARM_TEACH_RECORD) {
        dbg_print("[YAW] %s rejected: arm released for drag-teach\r\n", cmd);
        return -2;
    }
    if (st == ARM_TEACH_PLAYBACK) {
        dbg_print("[YAW] %s rejected: playback running ('teach stop' first)\r\n", cmd);
        return -3;
    }
    if (need_dir != 0U && arm_ctrl.yaw.dir_valid == 0U) {
        dbg_print("[YAW] %s rejected: direction not calibrated, send 'yaw probe' first\r\n", cmd);
        return -4;
    }
    if (arm_ctrl.yaw.fault != 0U) {
        dbg_print("[YAW] %s rejected: fault %u locked, send 'yaw fault clear'\r\n",
                  cmd, (unsigned)arm_ctrl.yaw.fault);
        return -5;
    }
    if (arm_motion_allowed() == 0U) {
        dbg_print("[YAW] %s rejected: mode=%s, send 'mode manual' first\r\n", cmd, arm_mode_name());
        return -6;
    }
    return 0;
}

static void yaw_fault(uint8_t code, const char *why)
{
    arm_ctrl.yaw.fault = code;
    arm_ctrl.yaw.enable = 0U;
    PID_clear(&arm_ctrl.yaw.angle_pid);
    PID_clear(&arm_ctrl.yaw.spd_pid);
    arm_ctrl.yaw.out = 0.0f;
    arm_ctrl.yaw.out_pid = 0.0f;
    arm_yaw_motor.tx_current = 0;
    dji_send_current(&arm_yaw_motor, NULL, NULL, NULL);
    s_last_send_tick = now_ms();
    s_runaway_tick = 0U;
    s_target_valid = 0U;
    s_yaw_report_force = 1U;
    dbg_print("[YAW] FAULT %u (%s): output zero, 'yaw fault clear' to reset\r\n",
              (unsigned)code, why);
    if (arm_ctrl.teach.state == ARM_TEACH_PLAYBACK) teach_abort("yaw fault");
}

int arm_yaw_set_enable(uint8_t en)
{
    uint8_t want = en ? 1U : 0U;

    if (want != 0U) {
        int rc = yaw_gate("enable", 1U);
        if (rc != 0) return rc;
        s_yaw_cmd_tick = now_ms();
        s_last_calc_tick = s_yaw_cmd_tick;
        s_runaway_tick = 0U;
        if (s_target_valid == 0U) {
            arm_ctrl.yaw.target_ecd = clamp_ecd((int32_t)arm_yaw_motor.ecd);
        }
    } else {
        PID_clear(&arm_ctrl.yaw.angle_pid);
        PID_clear(&arm_ctrl.yaw.spd_pid);
        arm_yaw_motor.tx_current = 0;
        dji_send_current(&arm_yaw_motor, NULL, NULL, NULL);   /* 立即零输出，不等下周期 */
        s_last_send_tick = now_ms();
        s_target_valid = 0U;
        s_runaway_tick = 0U;
    }
    if (arm_ctrl.yaw.enable != want) dbg_print("[YAW] enable=%u\r\n", (unsigned)want);
    arm_ctrl.yaw.enable = want;
    s_yaw_report_force = 1U;
    return 0;
}

int arm_yaw_set_target_ecd(int32_t ecd)
{
    int rc = yaw_gate("target", 1U);
    int32_t t;

    if (rc != 0) return rc;
    t = clamp_ecd(ecd);
    arm_ctrl.yaw.target_ecd = t;
    arm_ctrl.yaw.enable = 1U;
    s_target_valid = 1U;
    s_yaw_cmd_tick = now_ms();
    s_last_calc_tick = s_yaw_cmd_tick;
    s_runaway_tick = 0U;
    s_yaw_report_force = 1U;
    if (t != ecd) {
        dbg_print("[YAW] target clamped %ld -> %ld (limits %d/%d)\r\n",
                  (long)ecd, (long)t, ARM_YAW_ECD_LEFT_DEFAULT, ARM_YAW_ECD_RIGHT_DEFAULT);
    }
    return (int)t;
}

/* 开环给一个小电压看 ECD 往哪边走，据此标定 dir。环是断开的，接反也不会失控。 */
int arm_yaw_probe(void)
{
    int32_t e0, d, v;
    uint32_t t0;
    int rc = yaw_gate("probe", 0U);

    if (rc != 0) return rc;
    if (arm_ctrl.yaw.online == 0U) {
        dbg_print("[YAW] probe rejected: no feedback (on=0)\r\n");
        return -7;
    }

    arm_yaw_set_enable(0U);
    e0 = (int32_t)arm_yaw_motor.ecd;
    v = (arm_ctrl.yaw.out_limit < ARM_YAW_PROBE_OUT) ? (int32_t)arm_ctrl.yaw.out_limit : ARM_YAW_PROBE_OUT;
    dbg_print("[YAW] probe: +%ld for %ums, open loop\r\n", (long)v, (unsigned)ARM_YAW_PROBE_MS);
    t0 = now_ms();
    while ((uint32_t)(now_ms() - t0) < ARM_YAW_PROBE_MS) {
        arm_yaw_motor.tx_current = (int16_t)v;
        dji_send_current(&arm_yaw_motor, NULL, NULL, NULL);
        osDelay(5);
    }
    arm_yaw_motor.tx_current = 0;
    dji_send_current(&arm_yaw_motor, NULL, NULL, NULL);
    s_last_send_tick = now_ms();
    osDelay(60);                       /* 等它滑行停下再读 */

    d = (int32_t)arm_yaw_motor.ecd - e0;
    if (d > 4096) d -= 8192;
    else if (d < -4096) d += 8192;
    if (d >= 3 || d <= -3) {
        arm_ctrl.yaw.dir = (d > 0) ? 1 : -1;
        arm_ctrl.yaw.dir_valid = 1U;
        dbg_print("[YAW] probe: +%ld -> dECD=%ld, dir=%d\r\n",
                  (long)v, (long)d, (int)arm_ctrl.yaw.dir);
        return (int)arm_ctrl.yaw.dir;
    }
    dbg_print("[YAW] probe: no motion (dECD=%ld). Check 24V/ID, or raise 'yaw limit' (now %d)\r\n",
              (long)d, (int)arm_ctrl.yaw.out_limit);
    return 0;
}

int arm_yaw_set_dir(int32_t dir)
{
    if (dir != 1 && dir != -1) return -1;
    if (arm_ctrl.teach.state == ARM_TEACH_PLAYBACK) return -2;
    arm_ctrl.yaw.dir = (int8_t)dir;
    arm_ctrl.yaw.dir_valid = 1U;
    dbg_print("[YAW] dir=%d (set manually)\r\n", (int)dir);
    return 0;
}

int arm_yaw_set_limit(int32_t limit)
{
    if (limit < 500 || limit > ARM_YAW_OUT_LIMIT) return -1;
    if (arm_ctrl.teach.state == ARM_TEACH_PLAYBACK) return -2;
    arm_ctrl.yaw.out_limit = (int16_t)limit;
    dbg_print("[YAW] out_limit=%d (0x1FF voltage counts, hard cap %d)\r\n",
              (int)limit, ARM_YAW_OUT_LIMIT);
    return 0;
}

int arm_yaw_set_speed_limit(int32_t rpm)
{
    if (rpm < 10 || rpm > 3000) return -1;
    if (arm_ctrl.teach.state == ARM_TEACH_PLAYBACK) return -2;
    arm_ctrl.yaw.angle_pid.MaxOutput = (uint32_t)rpm;
    dbg_print("[YAW] position loop output limit=%ld rpm\r\n", (long)rpm);
    return 0;
}

int arm_yaw_set_gain(float pos_kp, float pos_kd, float spd_kp, float spd_kd)
{
    if (pos_kp < 0.0f || pos_kp > 1000.0f || pos_kd < 0.0f || pos_kd > 1000.0f ||
        spd_kp < 0.0f || spd_kp > 2000.0f || spd_kd < 0.0f || spd_kd > 1000.0f) return -1;
    if (arm_ctrl.teach.state == ARM_TEACH_PLAYBACK) return -2;
    arm_ctrl.yaw.angle_pid.p = pos_kp;
    arm_ctrl.yaw.angle_pid.d = pos_kd;
    arm_ctrl.yaw.spd_pid.p = spd_kp;
    arm_ctrl.yaw.spd_pid.d = spd_kd;
    dbg_print("[YAW] gain: pos kp=%.2f kd=%.2f, spd kp=%.2f kd=%.2f\r\n",
              (double)pos_kp, (double)pos_kd, (double)spd_kp, (double)spd_kd);
    return 0;
}

void arm_yaw_fault_clear(void)
{
    arm_ctrl.yaw.fault = 0U;
    s_runaway_tick = 0U;
    s_yaw_report_force = 1U;
    dbg_print("[YAW] fault cleared\r\n");
}

static void yaw_report(uint32_t now)
{
    uint8_t need = 0U;

    if (arm_ctrl.yaw.enable != 0U) {
        if ((uint32_t)(now - s_last_report_tick) >= 50U) need = 1U;    /* 20Hz */
    } else if (s_yaw_report_force != 0U ||
               (uint32_t)(now - s_last_report_tick) >= 1000U) {
        need = 1U;
    }
    if (need == 0U) return;

    s_yaw_report_force = 0U;
    s_last_report_tick = now;
    dbg_print("[YAW] en=%u on=%u flt=%u dir=%d tgt=%ld fdb=%ld err=%ld spd=%d/%.0f out=%.0f lim=%d side=%d\r\n",
              (unsigned)arm_ctrl.yaw.enable, (unsigned)arm_ctrl.yaw.online,
              (unsigned)arm_ctrl.yaw.fault, (int)arm_ctrl.yaw.dir,
              (long)arm_ctrl.yaw.target_ecd, (long)arm_ctrl.yaw.fdb_ecd,
              (long)arm_ctrl.yaw.err_ecd,
              (int)arm_ctrl.yaw.spd_fdb, arm_ctrl.yaw.spd_ref,
              arm_ctrl.yaw.out, (int)arm_ctrl.yaw.out_limit,
              (int)arm_yaw_limit_side());
}

void arm_yaw_pid_calc(void)
{
    uint32_t now = now_ms(), dt;
    int32_t ecd, ae;
    float spd, as, dsign;
    uint8_t enable, skip_d = 0U;

    /* 反馈新鲜度：计数变化即刷新 */
    if ((int32_t)arm_yaw_motor.receive_cnt != s_last_rx_cnt) {
        s_last_rx_cnt = (int32_t)arm_yaw_motor.receive_cnt;
        s_last_rx_tick = now;
    }
    arm_ctrl.yaw.online = ((uint32_t)(now - s_last_rx_tick) < ARM_YAW_FB_TIMEOUT_MS) ? 1U : 0U;

    ecd = (int32_t)arm_yaw_motor.ecd;
    if (s_target_valid == 0U) arm_ctrl.yaw.target_ecd = clamp_ecd(ecd);
    arm_ctrl.yaw.fdb_ecd = ecd;
    arm_ctrl.yaw.err_ecd = arm_ctrl.yaw.target_ecd - ecd;
    ae = iabs32(arm_ctrl.yaw.err_ecd);
    if (arm_ctrl.teach.state == ARM_TEACH_PLAYBACK && ae > arm_ctrl.teach.play_max_err) {
        arm_ctrl.teach.play_max_err = ae;
    }

    enable = arm_ctrl.yaw.enable;
    if (enable != 0U && arm_motion_allowed() == 0U) {
        /* 模式离开允许集合（拨杆回保护、mode protect）：立即停，且不自动恢复 */
        enable = 0U;
        arm_ctrl.yaw.enable = 0U;
        s_target_valid = 0U;
        PID_clear(&arm_ctrl.yaw.angle_pid);
        PID_clear(&arm_ctrl.yaw.spd_pid);
        dbg_print("[YAW] mode=%s -> zero output\r\n", arm_mode_name());
    }
    if (enable != 0U && arm_ctrl.yaw.fault != 0U) enable = 0U;
    if (enable != 0U && arm_ctrl.teach.state != ARM_TEACH_PLAYBACK) {
        if ((uint32_t)(now - s_yaw_cmd_tick) > ARM_YAW_CMD_HOLD_MS) {
            enable = 0U;
            arm_ctrl.yaw.enable = 0U;
            s_target_valid = 0U;
            dbg_print("[YAW] command timeout -> zero\r\n");
        }
    }
    if (arm_ctrl.teach.state == ARM_TEACH_READY || arm_ctrl.teach.state == ARM_TEACH_RECORD) {
        enable = 0U;
    }

    if (enable == 0U || arm_ctrl.yaw.online == 0U) {
        arm_ctrl.yaw.out = 0.0f;
        arm_ctrl.yaw.out_pid = 0.0f;
        arm_ctrl.yaw.spd_ref = 0.0f;
        arm_ctrl.yaw.spd_fdb = (float)arm_yaw_motor.speed_rpm;
        /* 零输出保活 50Hz */
        if ((uint32_t)(now - s_last_send_tick) >= 20U) {
            s_last_send_tick = now;
            arm_yaw_motor.tx_current = 0;
            dji_send_current(&arm_yaw_motor, NULL, NULL, NULL);
        }
        if (enable != 0U && arm_ctrl.yaw.online == 0U) {
            PID_clear(&arm_ctrl.yaw.angle_pid);
            PID_clear(&arm_ctrl.yaw.spd_pid);
            arm_ctrl.yaw.enable = 0U;
            s_target_valid = 0U;
            if (arm_ctrl.teach.state == ARM_TEACH_PLAYBACK) {
                arm_ctrl.yaw.fault = ARM_YAW_FAULT_FB_LOST;
                teach_abort("yaw feedback lost");
            } else {
                dbg_print("[YAW] feedback lost -> zero\r\n");
            }
        }
        s_last_calc_tick = now;
        yaw_report(now);
        return;
    }

    /* 长间隔（读舵机、写 Flash）后的第一次采样不做微分，避免速度环被踢 */
    dt = now - s_last_calc_tick;
    s_last_calc_tick = now;
    if (dt > (uint32_t)ARM_TASK_PERIOD * 2U) skip_d = 1U;

    arm_ctrl.yaw.angle_ref = (float)arm_ctrl.yaw.target_ecd * ARM_YAW_DEG_PER_ECD;
    arm_ctrl.yaw.angle_fdb = (float)ecd * ARM_YAW_DEG_PER_ECD;
    if (skip_d != 0U) arm_ctrl.yaw.angle_pid.err[PID_ERR_LAST] = arm_ctrl.yaw.angle_ref - arm_ctrl.yaw.angle_fdb;
    arm_ctrl.yaw.spd_ref = pid_calc(&arm_ctrl.yaw.angle_pid,
                                    arm_ctrl.yaw.angle_fdb,
                                    arm_ctrl.yaw.angle_ref);

    spd = (float)arm_yaw_motor.speed_rpm;
    arm_ctrl.yaw.spd_fdb = spd;
    if (skip_d != 0U) arm_ctrl.yaw.spd_pid.err[PID_ERR_LAST] = arm_ctrl.yaw.spd_ref - spd;
    arm_ctrl.yaw.out_pid = pid_calc(&arm_ctrl.yaw.spd_pid, spd, arm_ctrl.yaw.spd_ref);

    dsign = (float)arm_ctrl.yaw.dir;
    arm_ctrl.yaw.out = dsign * arm_ctrl.yaw.out_pid;

    /* 边界：只允许向区间内。out*dir>0 表示这个电压会让 ECD 增大 */
    if (ecd >= ARM_YAW_ECD_RIGHT_DEFAULT && arm_ctrl.yaw.out * dsign > 0.0f) {
        arm_ctrl.yaw.out = 0.0f;
    }
    if (ecd <= ARM_YAW_ECD_LEFT_DEFAULT && arm_ctrl.yaw.out * dsign < 0.0f) {
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

    /* 失控保护：误差与转速反向 = 正在远离目标（方向接反时的正反馈） */
    as = (spd < 0.0f) ? -spd : spd;
    if ((float)arm_ctrl.yaw.err_ecd * spd < 0.0f &&
        ae >= ARM_YAW_RUNAWAY_ERR_ECD && as >= ARM_YAW_RUNAWAY_SPD_RPM) {
        if (s_runaway_tick == 0U) {
            s_runaway_tick = now;
        } else if ((uint32_t)(now - s_runaway_tick) >= ARM_YAW_RUNAWAY_MS) {
            yaw_fault(ARM_YAW_FAULT_RUNAWAY, "moving away from target");
            yaw_report(now);
            return;
        }
    } else {
        s_runaway_tick = 0U;
    }
    yaw_report(now);
}

/* ========================= 点动 / 停止 ========================= */
int arm_joint_jog(uint8_t joint, const char *value)
{
    arm_teach_state_e st = arm_ctrl.teach.state;
    int32_t want;
    uint16_t pulse;
    char *end = NULL;
    long v;

    if (joint > 2U || value == NULL) return -1;
    if (servo_config_active()) {
        dbg_print("[JOG] rejected: servo config active ('servo run' first)\r\n");
        return -5;
    }
    if (arm_motion_allowed() == 0U) {
        dbg_print("[JOG] rejected: mode=%s, send 'mode manual' first\r\n", arm_mode_name());
        return -2;
    }
    if (st == ARM_TEACH_PLAYBACK) {
        dbg_print("[JOG] rejected: playback running ('teach stop' first)\r\n");
        return -3;
    }
    if ((st == ARM_TEACH_READY || st == ARM_TEACH_RECORD) && joint != 2U) {
        dbg_print("[JOG] rejected: shoulder/elbow released for drag-teach (only gripper allowed)\r\n");
        return -4;
    }

    v = strtol(value, &end, 10);
    if (end == value || *end != '\0') return -1;
    if (value[0] == '+' || value[0] == '-') {
        want = (int32_t)servo_get_current_pulse(joint) + (int32_t)v;
    } else {
        want = (int32_t)v;
    }
    if (want < 0) want = 0;
    if (want > 65535) want = 65535;
    pulse = joint_clamp(joint, (uint16_t)want);

    servo_set_enabled(1U);
    servo_set_pulse(joint, pulse);
    dbg_print("[JOG] %s pulse=%u (limits %u..%u)%s\r\n", joint_name(joint), (unsigned)pulse,
              (unsigned)s_joint_min[joint], (unsigned)s_joint_max[joint],
              (pulse != (uint16_t)want) ? " CLAMPED" : "");
    return 0;
}

int arm_joint_set_limit(uint8_t joint, int32_t lo, int32_t hi)
{
    if (joint > 2U) return -1;
    if (lo < (int32_t)SERVO_PULSE_MIN || hi > (int32_t)SERVO_PULSE_MAX || lo >= hi) {
        dbg_print("[JOG] limit rejected: need %u <= min < max <= %u\r\n",
                  (unsigned)SERVO_PULSE_MIN, (unsigned)SERVO_PULSE_MAX);
        return -2;
    }
    s_joint_min[joint] = (uint16_t)lo;
    s_joint_max[joint] = (uint16_t)hi;
    dbg_print("[JOG] %s limits %u..%u us (firmware does not know the mechanical stops)\r\n",
              joint_name(joint), (unsigned)lo, (unsigned)hi);
    return 0;
}

void arm_joint_status(void)
{
    uint8_t j;

    for (j = 0U; j < 3U; j++) {
        dbg_print("[JOG] %s pulse=%u limits=%u..%u online=%u\r\n", joint_name(j),
                  (unsigned)servo_get_current_pulse(j), (unsigned)s_joint_min[j],
                  (unsigned)s_joint_max[j], (unsigned)servo_online(j));
    }
}

void arm_stop_all(void)
{
    if (arm_ctrl.teach.state != ARM_TEACH_IDLE) teach_exit("stop");

    arm_yaw_set_enable(0U);
    arm_yaw_motor.tx_current = 0;
    dji_send_current(&arm_yaw_motor, NULL, NULL, NULL);
    if (mode_user_active() != 0U) mode_user_clear();
    arm_busy = 0U;
    dbg_print("[ARM] stop: yaw=0, servos hold, mode=PROTECT\r\n");
}

/* ========================= 控制台：teach ========================= */
void arm_teach_cmd(const char *sub, const char *arg)
{
    arm_teach_state_e st = arm_ctrl.teach.state;
    int rc;

    if (sub == NULL) return;

    if (strcmp(sub, "status") == 0) {
        uint32_t ms = 0U;
        uint16_t i;
        for (i = 1U; i < arm_teach_count; i++) {
            ms += (uint32_t)arm_teach_points[i].dt50 * TEACH_TICK_MS;
        }
        dbg_print("[TEACH] state=%s pts=%u dur=%lums origin=%u(%ld) saved=%u flash=%u seq=%lu slots=A:0x%08lX B:0x%08lX\r\n",
                  arm_teach_state_name(), (unsigned)arm_teach_count, (unsigned long)ms,
                  (unsigned)arm_ctrl.teach.origin_valid, (long)arm_ctrl.teach.origin_ecd,
                  (unsigned)arm_ctrl.teach.saved_valid, (unsigned)s_flash_ok,
                  (unsigned long)s_flash_seq, (unsigned long)TEACH_FLASH_A_ADDR,
                  (unsigned long)TEACH_FLASH_B_ADDR);
        return;
    }
    if (strcmp(sub, "pt") == 0) {
        char *end = NULL;
        long idx = (arg != NULL) ? strtol(arg, &end, 10) : -1;
        const arm_teach_point_t *p;
        if (arg == NULL || end == arg || *end != '\0' || idx < 0 || idx >= (long)arm_teach_count) {
            dbg_print("[TEACH] pt: index 0..%u\r\n",
                      (unsigned)((arm_teach_count > 0U) ? (arm_teach_count - 1U) : 0U));
            return;
        }
        p = &arm_teach_points[idx];
        dbg_print("[TEACH] pt %ld: yaw=%d sh=%u el=%u gr=%u dt=%ums\r\n", idx,
                  (int)p->yaw_ecd, (unsigned)p->shoulder, (unsigned)p->elbow,
                  (unsigned)p->gripper, (unsigned)((uint32_t)p->dt50 * TEACH_TICK_MS));
        return;
    }
    if (strcmp(sub, "enter") == 0) {
        teach_enter();
    } else if (strcmp(sub, "exit") == 0) {
        if (st != ARM_TEACH_IDLE) teach_exit("cmd");
        else dbg_print("[TEACH] already IDLE\r\n");
    } else if (strcmp(sub, "record") == 0) {
        if (st == ARM_TEACH_READY) teach_record_start();
        else dbg_print("[TEACH] record rejected: state=%s (need READY)\r\n", state_name(st));
    } else if (strcmp(sub, "save") == 0) {
        if (st == ARM_TEACH_RECORD) teach_save();
        else dbg_print("[TEACH] save rejected: state=%s (need RECORD)\r\n", state_name(st));
    } else if (strcmp(sub, "play") == 0) {
        if (st == ARM_TEACH_SAVED) teach_play_start();
        else dbg_print("[TEACH] play rejected: state=%s (need SAVED)\r\n", state_name(st));
    } else if (strcmp(sub, "stop") == 0) {
        if (st == ARM_TEACH_PLAYBACK) teach_play_stop("cmd");
        else dbg_print("[TEACH] stop ignored: state=%s (use 'stop' to leave teach)\r\n", state_name(st));
    } else if (strcmp(sub, "load") == 0) {
        if (st == ARM_TEACH_READY || st == ARM_TEACH_RECORD || st == ARM_TEACH_PLAYBACK) {
            dbg_print("[TEACH] load rejected while %s\r\n", state_name(st));
            return;
        }
        rc = flash_load();
        dbg_print("[TEACH] load rc=%d pts=%u\r\n", rc, (unsigned)arm_teach_count);
        if (rc == 0) teach_set_state(ARM_TEACH_SAVED, "load");
    } else if (strcmp(sub, "erase") == 0) {
        teach_erase();
    } else if (strcmp(sub, "origin") == 0) {
        arm_ctrl.teach.origin_valid = 1U;
        arm_ctrl.teach.origin_ecd = (int32_t)arm_yaw_motor.ecd;
        dbg_print("[TEACH] origin ecd=%ld\r\n", (long)arm_ctrl.teach.origin_ecd);
    } else {
        dbg_print("[TEACH] usage: teach status|pt <n>|enter|exit|record|save|play|stop|load|erase|origin\r\n");
    }
}

/* ========================= 初始化 / 主循环 ========================= */
void arm_task_init(void)
{
    uint32_t now = now_ms();

    memset(&arm_ctrl, 0, sizeof(arm_ctrl));
    BSP_UserKey_Init();
    servo_init();

    /* 双环参数沿用 155ab05，未整定；Ki 都是 0 */
    PID_struct_init(&arm_ctrl.yaw.angle_pid, 500U, 200U,
                    40.0f, 0.0f, 5.0f, 0.0f, 0.0f, 0.0f, NONE_IMPROVE);
    PID_struct_init(&arm_ctrl.yaw.spd_pid, (uint32_t)ARM_YAW_OUT_LIMIT, 10000U,
                    198.0f, 0.0f, 13.0f, 0.0f, 0.0f, 0.0f, NONE_IMPROVE);
    arm_ctrl.yaw.out_limit = ARM_YAW_OUT_LIMIT_DFLT;
    arm_ctrl.yaw.dir = 1;
    arm_ctrl.yaw.dir_valid = 0U;       /* 上电必须先 'yaw probe' */
    arm_ctrl.yaw.target_ecd = clamp_ecd((int32_t)arm_yaw_motor.ecd);

    s_joint_min[0] = ARM_JOINT_DFLT_MIN;
    s_joint_max[0] = ARM_JOINT_DFLT_MAX;
    s_joint_min[1] = ARM_JOINT_DFLT_MIN;
    s_joint_max[1] = ARM_JOINT_DFLT_MAX;
    s_joint_min[2] = ARM_GRIPPER_DFLT_MIN;
    s_joint_max[2] = ARM_GRIPPER_DFLT_MAX;

    s_last_rx_cnt = (int32_t)arm_yaw_motor.receive_cnt;
    s_last_rx_tick = now;
    s_last_send_tick = now;
    s_last_report_tick = now;
    s_last_calc_tick = now;
    s_yaw_cmd_tick = now;
    s_runaway_tick = 0U;
    s_target_valid = 0U;
    s_yaw_report_force = 1U;
    s_sample_gap = 0U;
    s_play_phase = 0U;

    /* Flash 镜像按 10 字节一个点排布，布局不成立就不启用 */
    s_flash_ok = (sizeof(arm_teach_point_t) == 10U) ? 1U : 0U;
    if (s_flash_ok != 0U && flash_load() == 0) {
        dbg_print("[ARM] flash track loaded: pts=%u seq=%lu (double click to arm playback)\r\n",
                  (unsigned)arm_teach_count, (unsigned long)s_flash_seq);
    }

    dbg_print("[ARM] init: yaw limits=%d/%d out_limit=%d dir=uncalibrated ('yaw probe'), joint sh/el %u..%u gr %u..%u, teach flash ok=%u\r\n",
              ARM_YAW_ECD_LEFT_DEFAULT, ARM_YAW_ECD_RIGHT_DEFAULT, (int)arm_ctrl.yaw.out_limit,
              ARM_JOINT_DFLT_MIN, ARM_JOINT_DFLT_MAX, ARM_GRIPPER_DFLT_MIN, ARM_GRIPPER_DFLT_MAX,
              (unsigned)s_flash_ok);

    vision_uart_init();        /* UART8/PE0 视觉 RX，PC USB-TTL 可直接模拟 */

    arm_yaw_motor.tx_current = 0;
    dji_send_current(&arm_yaw_motor, NULL, NULL, NULL);
}

void arm_task(void const *argu)
{
    uint32_t wake;

    (void)argu;
    arm_task_init();
    wake = now_ms();

    for (;;) {
        key_event_e ev;

        servo_poll();              /* 总线舵机解析 + 执行一条控制台命令 */
        servo_test_poll();         /* yaw monitor（旧遥测） */
        ev = BSP_UserKey_Poll();
        if (ev != KEY_EVT_NONE) handle_key(ev);
        teach_tick();
        arm_yaw_pid_calc();

        osDelayUntil(&wake, ARM_TASK_PERIOD);
        if ((uint32_t)(now_ms() - wake) > 50U) {
            wake = now_ms();       /* 长阻塞（Flash、舵机读）后重新对齐，避免空转追赶 */
        }
    }
}

/* prot_vision.c 的回调占位 */
void arm_vision_callback(void)
{
}
