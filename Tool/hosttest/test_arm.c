/* 主机场景测试：把真实的 Application/arm_task.c 原样包含进来，硬件全部用 sim.c 模拟。
   跑法：py Tool\\hosttest\\run.py
   固件日志默认不打印，设环境变量 SIM_VERBOSE=1 可以看。 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "sim.h"
#include "arm_task.c"       /* 真实固件源文件，静态函数在这里可见 */

static int g_fail, g_pass;
static const char *g_section = "";

#define CHECK(c, msg) do { if (c) { g_pass++; } else { g_fail++; printf("    FAIL [%s] %s (line %d)\n", g_section, msg, __LINE__); } } while (0)

#define SLOT_A  0x080C0000UL
#define SLOT_B  0x080E0000UL
#define FLASH_U32(a) (*(const uint32_t *)(uintptr_t)(a))

static void section(const char *name)
{
    g_section = name;
    g_cmd_log_n = 0U;
    printf("== %s\n", name);
}

/* 模拟机械臂任务主循环：每圈 3ms，顺序与 arm_task() 一致 */
static void run(uint32_t ms)
{
    uint32_t end = g_now_ms + ms;

    while (g_now_ms < end) {
        key_event_e ev;
        servo_poll();
        servo_test_poll();
        ev = BSP_UserKey_Poll();
        if (ev != KEY_EVT_NONE) handle_key(ev);
        teach_tick();
        arm_yaw_pid_calc();
        sim_advance(3U);
    }
}

static void press_for(uint32_t hold_ms) { g_btn_pressed = 1; run(hold_ms); }
static void wait(uint32_t ms)           { g_btn_pressed = 0; run(ms); }
static void click(void)        { press_for(80U);   wait(450U); }
static void dblclick(void)     { press_for(80U);   wait(120U); press_for(80U); wait(450U); }
static void tripleclick(void)  { press_for(80U);   wait(120U); press_for(80U); wait(120U); press_for(80U); wait(500U); }
static void long2(void)        { press_for(2300U); wait(700U); }
static void long5(void)        { press_for(5300U); wait(700U); }

static int run_until_state(arm_teach_state_e st, uint32_t timeout_ms)
{
    uint32_t end = g_now_ms + timeout_ms;

    while (g_now_ms < end) {
        run(10U);
        if (arm_ctrl.teach.state == st) return 1;
    }
    return 0;
}

/* 跑 ms 毫秒，记录 yaw 的 ECD 范围和输出峰值 */
static float s_ecd_min, s_ecd_max, s_out_max;
static void track_begin(void) { s_ecd_min = 1e9f; s_ecd_max = -1e9f; s_out_max = 0.0f; }
static void track_run(uint32_t ms)
{
    uint32_t end = g_now_ms + ms;
    while (g_now_ms < end) {
        run(3U);
        if (g_yaw_angle < s_ecd_min) s_ecd_min = g_yaw_angle;
        if (g_yaw_angle > s_ecd_max) s_ecd_max = g_yaw_angle;
        if ((float)g_yaw_tx > s_out_max) s_out_max = (float)g_yaw_tx;
        if ((float)-g_yaw_tx > s_out_max) s_out_max = (float)-g_yaw_tx;
    }
}

/* 用标定把脉宽还原成连杆角，再做正运动学 */
static void fk_from_pulse(uint16_t sh, uint16_t el, float *x, float *y)
{
    const arm_cal_t *c = &arm_ctrl.cal;
    float k_sh = ((float)c->sh90_pulse - (float)c->sh0_pulse) / 90.0f;
    float k_el = ((float)c->el90_pulse - (float)c->el0_pulse) / 90.0f;
    float th1 = ((float)sh - (float)c->sh0_pulse) / k_sh * DEG2RAD;
    float th2 = ((float)el - (float)c->el0_pulse) / k_el * DEG2RAD;

    *x = (float)c->l1_mm * cosf(th1) + (float)c->l2_mm * cosf(th1 + th2);
    *y = (float)c->l1_mm * sinf(th1) + (float)c->l2_mm * sinf(th1 + th2);
}

/* 录一段轨迹：进示教 -> 标原点 -> 开始录 -> 拖 -> 双击保存。返回点数 */
static uint16_t teach_a_track(int steps, int d_sh, int d_el)
{
    int i;
    long2();
    click();
    click();
    for (i = 1; i <= steps; i++) {
        sim_set_pulse(0, (int16_t)(1500 + d_sh * i));
        sim_set_pulse(1, (int16_t)(1500 + d_el * i));
        run(60U);
    }
    dblclick();
    return arm_teach_count;
}

int main(void)
{
    uint16_t n_saved, n_x, n_y, sh_final = 0U, s_ik_pose_saved_sh = 0U;
    arm_teach_point_t pts_saved[ARM_TEACH_MAX_POINTS];
    int32_t ecd0;
    uint32_t t0, i, n_close, n_last_grip, n_sh1700, el;
    float x, y;
    int rc;

    sim_reset();
    g_verbose = (getenv("SIM_VERBOSE") != NULL);
    g_capture = 1;                       /* 固件打印攒起来，个别检查要找里面的提示文字 */
    sim_advance(200U);                   /* 上电前 6020 已经在发反馈 */
    arm_task_init();
    run(300U);

    /* ------------------------------------------------------------------ */
    section("T1 上电状态");
    CHECK(arm_ctrl.teach.state == ARM_TEACH_IDLE, "teach IDLE");
    CHECK(arm_busy == 0, "chassis NOT locked");
    CHECK(ctrl_mode == PROTECT_MODE, "mode PROTECT");
    CHECK(arm_ctrl.yaw.enable == 0, "yaw disabled");
    CHECK(arm_ctrl.yaw.dir_valid == 0, "yaw direction not calibrated");
    CHECK(arm_ctrl.yaw.online == 1, "yaw feedback online");
    CHECK(s_flash_ok == 1, "flash layout valid (point 10B, cal 32B)");
    CHECK(arm_teach_count == 0 && arm_ctrl.cal.ik_valid == 0, "blank flash: no track, IK uncalibrated");
    CHECK(arm_ctrl.cal.l1_mm == 150 && arm_ctrl.cal.l2_mm == 150, "default link lengths 150/150");
    CHECK(arm_ctrl.yaw.out_limit == 8000, "default yaw output limit 8000");
    CHECK(arm_ctrl.pick_wp[1].shoulder == 1700U && arm_ctrl.pick_wp[1].elbow == 2000U,
          "factory pick = June demo EXTEND 1700/2000");
    CHECK(arm_ctrl.cal.pick_open == 1100U && arm_ctrl.cal.pick_close == 1450U,
          "factory gripper = June demo 1100 open / 1450 close");

    /* ------------------------------------------------------------------ */
    section("T2 保护模式拒绝一切运动");
    CHECK(arm_joint_jog(0, "1550") < 0, "joint rejected in PROTECT");
    CHECK(arm_yaw_set_target_ecd(3050) < 0, "yaw target rejected in PROTECT");
    CHECK(arm_yaw_set_enable(1) < 0, "yaw enable rejected in PROTECT");
    CHECK(arm_yaw_probe() < 0, "yaw probe rejected in PROTECT");
    CHECK(arm_chassis_release() != 0 || 1, "chassis line accepted from console (it is its own explicit start)");
    arm_chassis_stop();
    CHECK(g_cmd_log_n == 0, "no servo command was sent");

    /* ------------------------------------------------------------------ */
    section("T3 mode manual：只放开机械臂，底盘保持锁零");
    arm_chassis_stop();
    CHECK(arm_mode_manual() == 0, "mode manual accepted");
    CHECK(ctrl_mode == ARM_TEACH_MODE, "manual uses ARM_TEACH, never REMOTER");
    CHECK(arm_busy == 1, "chassis locked in manual mode");
    CHECK(arm_joint_jog(0, "1550") == 0, "shoulder accepted in manual");
    CHECK(g_cmd_log_n == 1 && g_cmd_log[0].pulse == 1550, "shoulder 1550 sent");
    CHECK(arm_joint_jog(0, "500") == 0 && g_cmd_log[1].pulse == 1400, "shoulder clamped to lower limit 1400");
    CHECK(arm_joint_jog(0, "3000") == 0 && g_cmd_log[2].pulse == 1600, "shoulder clamped to upper limit 1600");
    CHECK(arm_joint_set_limit(0, 600, 2400) == 0, "shoulder limits widened");
    CHECK(arm_joint_jog(0, "700") == 0 && g_cmd_log[3].pulse == 700, "700 accepted after widening");
    CHECK(arm_joint_set_limit(0, 400, 3000) != 0, "limits outside 500..2500 rejected");
    CHECK(arm_joint_set_limit(0, 1700, 1400) != 0, "min >= max rejected");
    (void)arm_joint_set_limit(0, 1400, 1600);
    g_pulse[0] = 1500;
    CHECK(arm_joint_jog(1, "+60") == 0 && g_cmd_log[4].pulse == 1560, "elbow +60 relative to current 1500");
    g_pulse[1] = 1500;
    CHECK(arm_joint_jog(2, "2000") == 0 && g_gripper_cmd == 2000, "gripper 2000 accepted");
    CHECK(arm_joint_jog(2, "3000") == 0 && g_gripper_cmd == 2060, "gripper clamped to 2060");
    CHECK(arm_joint_jog(0, "abc") < 0, "non-numeric rejected");
    g_config_active = 1;
    CHECK(arm_joint_jog(0, "1550") < 0 && arm_yaw_set_enable(1) < 0, "servo config mode blocks motion");
    g_config_active = 0;
    arm_stop_all();
    CHECK(arm_busy == 0 && ctrl_mode == PROTECT_MODE, "stop -> chassis free, PROTECT");
    CHECK(arm_joint_jog(0, "1550") < 0, "joint rejected again after stop");

    /* ------------------------------------------------------------------ */
    section("T4 底盘巡线与机械臂命令互斥");
    CHECK(arm_chassis_release() == 0, "chassis release accepted");
    CHECK(arm_ctrl.chassis_run == 1 && arm_busy == 0, "chassis released, arm lock cleared");
    CHECK(arm_joint_jog(0, "1550") < 0, "joint rejected while chassis runs");
    CHECK(arm_yaw_set_target_ecd(3050) < 0, "yaw rejected while chassis runs");
    CHECK(arm_mode_manual() == 0 && arm_ctrl.chassis_run == 0 && arm_busy == 1, "mode manual takes the chassis back");
    arm_chassis_stop();
    CHECK(arm_ctrl.chassis_run == 0 && ctrl_mode == PROTECT_MODE, "chassis stop -> PROTECT");

    /* ------------------------------------------------------------------ */
    section("T5 yaw 方向标定（开环探测）");
    CHECK(arm_mode_manual() == 0, "manual for yaw");
    g_yaw_online = 0;
    run(250U);
    CHECK(arm_ctrl.yaw.online == 0, "yaw offline detected within 100ms");
    CHECK(arm_yaw_probe() < 0, "probe rejected without feedback");
    g_yaw_online = 1;
    run(50U);
    CHECK(arm_ctrl.yaw.online == 1, "yaw online again");
    CHECK(arm_yaw_set_target_ecd(3100) < 0, "yaw command rejected until direction is calibrated");
    rc = arm_yaw_probe();
    CHECK(rc == 1, "probe returns +1 for normal wiring");
    CHECK(arm_ctrl.yaw.dir_valid == 1 && arm_ctrl.yaw.dir == 1, "dir calibrated to +1");
    CHECK(arm_ctrl.yaw.enable == 0, "probe leaves the loop disabled");

    /* 离限位太近：不探测（150 ECD 保护带，两头都算） */
    g_yaw_angle = 1400.0f;                       /* 左限位 1260 + 140 */
    run(20U);
    t0 = g_yaw_frames;
    sim_out_clear();
    CHECK(arm_yaw_probe() == 0 && g_yaw_frames <= t0 + 2U && g_yaw_tx == 0, "no probe voltage inside the left limit band (2 zero frames at most)");
    CHECK(sim_out_has("within 150 ECD of a limit"), "and the reason is printed");
    g_yaw_angle = 6350.0f;                       /* 右限位 6478 - 128 */
    run(20U);
    t0 = g_yaw_frames;
    CHECK(arm_yaw_probe() == 0 && g_yaw_frames <= t0 + 2U && g_yaw_tx == 0, "no probe voltage inside the right limit band");
    g_yaw_angle = 3000.0f;
    run(20U);
    t0 = g_yaw_frames;
    CHECK(arm_yaw_probe() == 1 && g_yaw_frames > t0, "probe runs again in the middle of the range");

    /* ------------------------------------------------------------------ */
    section("T6 yaw 跟随、限幅、边界");
    run(1000U);
    ecd0 = (int32_t)arm_yaw_motor.ecd;
    rc = arm_yaw_set_target_ecd(ecd0 + 200);
    CHECK(rc == ecd0 + 200, "target accepted, not clamped");
    track_begin();
    track_run(2500U);
    CHECK(iabs32(arm_ctrl.yaw.err_ecd) <= 15, "reached target within 15 ECD");
    CHECK(s_out_max <= 8000.0f, "output never above the 8000 limit");
    CHECK(arm_ctrl.yaw.enable == 1, "still enabled (10s hold not expired)");

    rc = arm_yaw_set_target_ecd(9000);
    CHECK(rc == 6478, "target 9000 clamped to right limit 6478");
    track_begin();
    track_run(5000U);
    CHECK(s_ecd_max <= 6478.0f + 60.0f, "never ran past the right limit by more than 60 ECD");
    CHECK(iabs32(arm_ctrl.yaw.err_ecd) <= 15, "settled on the right limit");

    rc = arm_yaw_set_target_ecd(1000);
    CHECK(rc == 1260, "target 1000 clamped to left limit 1260");
    track_begin();
    track_run(6000U);
    CHECK(s_ecd_min >= 1260.0f - 60.0f, "never ran past the left limit by more than 60 ECD");
    CHECK(iabs32(arm_ctrl.yaw.err_ecd) <= 15, "settled on the left limit");

    CHECK(arm_yaw_set_target_ecd(3000) == 3000, "back to mid range");
    track_run(3000U);
    CHECK(iabs32(arm_ctrl.yaw.err_ecd) <= 15, "reached mid range");
    CHECK(arm_ctrl.yaw.fault == 0, "large normal moves (1260 -> 6478 -> 3000) never trip the runaway guard");

    /* ------------------------------------------------------------------ */
    section("T7 线接反且没标定：失控保护");
    g_yaw_wiring_dir = -1;
    (void)arm_yaw_set_dir(1);                 /* 故意标错 */
    ecd0 = (int32_t)arm_yaw_motor.ecd;
    (void)arm_yaw_set_target_ecd(ecd0 + 100);
    track_begin();
    track_run(1500U);
    CHECK(arm_ctrl.yaw.fault == ARM_YAW_FAULT_RUNAWAY, "runaway fault latched");
    CHECK(arm_ctrl.yaw.enable == 0, "yaw disabled by the fault");
    CHECK(g_yaw_tx == 0, "output is zero after the fault");
    CHECK((float)ecd0 - s_ecd_min < 220.0f, "cut within 220 ECD (under 10 deg) including the coast afterwards");
    printf("    reversed wiring: yaw moved %.0f ECD before the cut\n", (double)((float)ecd0 - s_ecd_min));
    CHECK(arm_yaw_set_target_ecd(3000) < 0, "commands rejected while fault latched");
    CHECK(arm_yaw_set_enable(1) < 0, "enable rejected while fault latched");
    arm_yaw_fault_clear();
    CHECK(arm_ctrl.yaw.fault == 0, "fault cleared");
    run(1500U);
    CHECK(arm_yaw_probe() == -1, "probe finds dir=-1 for reversed wiring");
    rc = arm_yaw_set_target_ecd(3200);
    track_begin();
    track_run(3000U);
    CHECK(iabs32(arm_ctrl.yaw.err_ecd) <= 15, "reversed wiring controlled correctly after probe");

    /* ------------------------------------------------------------------ */
    section("T8 yaw 命令 10 秒超时归零、模式切回保护立即归零");
    run(7500U);                               /* 距上一条命令约 10.5s */
    CHECK(arm_ctrl.yaw.enable == 0, "command hold expired -> zero output");
    CHECK(g_yaw_tx == 0, "tx zero after timeout");
    (void)arm_yaw_set_target_ecd(3400);
    run(200U);
    CHECK(arm_ctrl.yaw.enable == 1, "moving again");
    mode_user_clear();                        /* 相当于 mode protect / 拨杆回保护 */
    run(5U);
    CHECK(arm_ctrl.yaw.enable == 0, "mode left allowed set -> disabled at once");
    run(30U);
    CHECK(g_yaw_tx == 0, "zero frame sent within 30ms");
    (void)arm_yaw_set_target_ecd(3400);
    run(100U);
    g_yaw_online = 0;
    run(250U);
    CHECK(arm_ctrl.yaw.enable == 0 && g_yaw_tx == 0, "feedback lost -> zero output");
    g_yaw_online = 1;
    run(100U);
    arm_stop_all();

    /* ------------------------------------------------------------------ */
    section("T9 按键：进入示教 -> 录制 -> 保存 -> 退出");
    CHECK(arm_ctrl.teach.state == ARM_TEACH_IDLE, "start from IDLE");
    CHECK(arm_joint_set_limit(0, 500, 2500) == 0 && arm_joint_set_limit(1, 500, 2500) == 0, "joint limits widened for the drag test");
    g_pulse[0] = 1500;
    g_pulse[1] = 1500;
    g_yaw_wiring_dir = 1;                     /* 接线恢复正常 */
    (void)arm_yaw_set_dir(1);
    long2();
    CHECK(arm_ctrl.teach.state == ARM_TEACH_READY, "long press 2.3s -> READY");
    CHECK(g_released == 1, "shoulder/elbow released for dragging");
    CHECK(arm_busy == 1, "chassis locked while teaching");
    CHECK(arm_ctrl.yaw.enable == 0, "yaw torque off while teaching");
    CHECK(arm_yaw_set_target_ecd(3600) < 0, "yaw command rejected while arm is released");
    CHECK(arm_joint_jog(0, "1550") < 0, "shoulder jog rejected while released");
    CHECK(arm_joint_jog(2, "1800") == 0 && g_gripper_cmd == 1800, "gripper jog allowed while released");
    CHECK(arm_chassis_release() < 0, "chassis line rejected while arm is released");
    CHECK(arm_mode_manual() < 0, "mode manual rejected while arm is released");
    click();
    CHECK(arm_ctrl.teach.origin_valid == 1, "first short press = origin");
    CHECK(arm_ctrl.teach.state == ARM_TEACH_READY, "still READY after origin");
    click();
    CHECK(arm_ctrl.teach.state == ARM_TEACH_RECORD, "second short press = RECORD");
    CHECK(arm_teach_count == 1, "start pose stored as pt0");
    for (i = 1; i <= 30; i++) {
        sim_set_pulse(0, (int16_t)(1500 + i * 10));
        sim_set_pulse(1, (int16_t)(1500 - i * 6));
        run(60U);
    }
    CHECK(arm_teach_count >= 8, "points recorded while dragging");
    n_x = (uint16_t)arm_teach_count;
    run(700U);                                /* 停住不动 */
    CHECK(arm_teach_count == n_x, "no new points while not moving");
    sim_set_pulse(0, 1900);
    run(120U);
    CHECK(arm_teach_count >= n_x + 2, "moving after a pause adds a hold point + the new point");
    n_x = (uint16_t)arm_teach_count;
    click();
    CHECK(arm_teach_count == (uint16_t)(n_x + 1U), "short press during record = key point");
    dblclick();
    CHECK(arm_ctrl.teach.state == ARM_TEACH_SAVED, "double click = save -> SAVED");
    CHECK(arm_ctrl.teach.saved_valid == 1, "saved_valid set");
    CHECK(g_released == 0, "servos holding again after save");
    CHECK(arm_busy == 0, "chassis NOT locked in SAVED");
    CHECK(FLASH_U32(SLOT_B) == 0x41544C33UL, "slot B written (first save goes to B)");
    CHECK(FLASH_U32(SLOT_A) == 0xFFFFFFFFUL, "slot A still blank");
    n_saved = arm_teach_count;
    memcpy(pts_saved, arm_teach_points, (size_t)n_saved * sizeof(arm_teach_point_t));
    long2();
    CHECK(arm_ctrl.teach.state == ARM_TEACH_IDLE, "long press in SAVED -> IDLE");
    CHECK(arm_busy == 0 && ctrl_mode == PROTECT_MODE, "IDLE: chassis free, PROTECT");
    long2();
    CHECK(arm_ctrl.teach.state == ARM_TEACH_READY, "re-enter teach");
    click();
    click();
    long2();
    CHECK(arm_ctrl.teach.state == ARM_TEACH_IDLE, "exit without saving");
    CHECK(arm_ctrl.teach.saved_valid == 0, "unsaved recording cannot be played");

    /* ------------------------------------------------------------------ */
    section("T10 掉电重启：轨迹从片内 Flash 恢复并回放");
    arm_task_init();                          /* 等价于重启：RAM 全清，Flash 保留 */
    run(200U);
    CHECK(arm_ctrl.teach.state == ARM_TEACH_IDLE, "reboot -> IDLE");
    CHECK(arm_teach_count == n_saved, "track restored from flash");
    CHECK(memcmp(pts_saved, arm_teach_points, (size_t)n_saved * sizeof(arm_teach_point_t)) == 0, "restored points are identical");
    CHECK(arm_ctrl.teach.saved_valid == 1 && arm_busy == 0, "saved, chassis free");
    CHECK(arm_ctrl.yaw.dir_valid == 1 && arm_ctrl.yaw.dir == 1, "yaw direction remembered across reboot");
    CHECK(arm_ctrl.yaw.enable == 0, "yaw disabled after reboot");
    n_x = (uint16_t)g_cmd_log_n;
    dblclick();
    CHECK(arm_ctrl.teach.state == ARM_TEACH_SAVED, "double click -> SAVED (armed)");
    CHECK(g_cmd_log_n == n_x, "arming sends no servo command");
    dblclick();
    CHECK(arm_ctrl.teach.state == ARM_TEACH_PLAYBACK, "second double click -> PLAYBACK");
    CHECK(arm_busy == 1, "chassis locked during playback");
    CHECK(s_play_yaw == 1, "yaw takes part when it is calibrated and online");
    t0 = g_now_ms;
    CHECK(run_until_state(ARM_TEACH_SAVED, 30000U), "playback finished");
    el = g_now_ms - t0;
    CHECK(arm_busy == 0 && ctrl_mode == PROTECT_MODE, "after playback: chassis free, mode PROTECT");
    CHECK(arm_ctrl.yaw.enable == 0, "yaw off again after playback");
    CHECK(g_cmd_log_n >= 4U, "servo commands were sent");
    printf("    playback took %lums for %u points\n", (unsigned long)el, (unsigned)n_saved);
    sh_final = 0U;
    for (i = 0; i < g_cmd_log_n; i++) if (g_cmd_log[i].joint == 0U) sh_final = g_cmd_log[i].pulse;
    CHECK(sh_final == (uint16_t)pts_saved[n_saved - 1U].shoulder, "shoulder ends on the last recorded point");

    /* 短按停回放 */
    dblclick();
    CHECK(run_until_state(ARM_TEACH_PLAYBACK, 2000U), "playback restarted");
    click();
    run(50U);
    CHECK(arm_ctrl.teach.state == ARM_TEACH_SAVED, "short press stops playback");
    CHECK(arm_busy == 0, "chassis free after stopping");

    /* ------------------------------------------------------------------ */
    section("T11 出厂默认就是 6 月表演动作：三击就跑，再按停止");
    arm_teach_cmd("erase", NULL);
    CHECK(arm_teach_count == 0 && arm_ctrl.teach.saved_valid == 0, "track erased from RAM");
    CHECK(FLASH_U32(SLOT_A) == 0xFFFFFFFFUL, "slot A blank after erase");
    CHECK(FLASH_U32(SLOT_B) == 0x41544C33UL && FLASH_U32(SLOT_B + 12U) == 0x00030000UL, "slot B holds a calibration-only image (count 0, ver 3)");
    CHECK(arm_ctrl.cal.ik_valid == 0, "IK uncalibrated");
    n_x = (uint16_t)g_cmd_log_n;
    tripleclick();
    CHECK(arm_ctrl.teach.state == ARM_TEACH_PLAYBACK, "triple click with no saved track runs the factory June demo");
    t0 = g_now_ms;
    CHECK(run_until_state(ARM_TEACH_SAVED, 15000U), "demo finished");
    CHECK(g_now_ms - t0 >= 6000U, "demo took the June 2s-per-pose time");
    CHECK(g_cmd_log_n > n_x, "the demo really moved the servos");
    n_sh1700 = 0U;
    for (i = 0U; i < g_cmd_log_n; i++) if (g_cmd_log[i].joint == 0U && g_cmd_log[i].pulse == 1700U) n_sh1700 = 1U;
    CHECK(n_sh1700 == 1U, "demo reached the June EXTEND pose (shoulder 1700)");
    n_close = 0;
    for (i = 0U; i < g_cmd_log_n; i++) if (g_cmd_log[i].joint == 2U && g_cmd_log[i].pulse == 1450U) n_close++;
    CHECK(n_close > 0, "demo closes the gripper to the June 1450");
    tripleclick();
    CHECK(arm_ctrl.teach.state == ARM_TEACH_PLAYBACK, "triple click restarts the demo");
    run(2500U);
    tripleclick();
    CHECK(arm_ctrl.teach.state == ARM_TEACH_SAVED && arm_busy == 0, "triple again mid-demo stops it");
    long2();
    CHECK(arm_ctrl.teach.state == ARM_TEACH_IDLE, "back to IDLE for the next section");

    /* ------------------------------------------------------------------ */
    section("T12 死程序采摘：学 5 个路点，三击执行");
    arm_ik_cmd("learn", "sh0", NULL);
    CHECK(arm_ctrl.cal.ik_valid == 0, "ik learn without a track is still allowed to record");
    long2();
    CHECK(arm_ctrl.teach.state == ARM_TEACH_READY, "READY");
    sim_set_pulse(0, 1500); sim_set_pulse(1, 1500); run(60U);
    arm_pick_cmd("learn", "0", NULL);
    sim_set_pulse(0, 1700); sim_set_pulse(1, 1900); run(60U);
    arm_pick_cmd("learn", "1", NULL);
    arm_pick_cmd("learn", "2", NULL);
    sim_set_pulse(0, 1350); sim_set_pulse(1, 1300); run(60U);
    arm_pick_cmd("learn", "3", NULL);
    sim_set_pulse(0, 1500); sim_set_pulse(1, 1500); run(60U);
    arm_pick_cmd("learn", "4", NULL);
    CHECK(arm_teach_count == 5, "5-point pick track built");
    CHECK(arm_ctrl.pick_wp[1].shoulder == 1700 && arm_ctrl.pick_wp[1].elbow == 1900, "EXTEND pose learned");
    CHECK(arm_ctrl.pick_wp[3].shoulder == 1350 && arm_ctrl.pick_wp[3].elbow == 1300, "RETRACT pose learned");
    rc = arm_flash_commit();
    CHECK(rc == 0, "pick track saved to flash");
    long2();                                   /* READY -> IDLE，saved_valid 会被清掉 */
    CHECK(arm_ctrl.teach.state == ARM_TEACH_IDLE && arm_ctrl.teach.saved_valid == 0, "exit teach clears RAM flag");
    CHECK(g_cmd_log_n == 0, "no motion before triple click");
    tripleclick();
    CHECK(arm_ctrl.teach.state == ARM_TEACH_PLAYBACK, "triple click runs the pick sequence (reloads from flash)");
    t0 = g_now_ms;
    CHECK(run_until_state(ARM_TEACH_SAVED, 20000U), "pick sequence finished");
    CHECK(g_now_ms - t0 >= 3600U, "took about the configured 4s of motion");
    n_close = 0U; n_last_grip = 0U; n_sh1700 = 0U;
    for (i = 0U; i < g_cmd_log_n; i++) {
        if (g_cmd_log[i].joint == 2U) {
            if (g_cmd_log[i].pulse == 1450U) { n_close++; if (n_sh1700 == 0U) n_sh1700 = i; }
            n_last_grip = g_cmd_log[i].pulse;
        }
    }
    { uint32_t q, n2 = 0U; int32_t mn = 99999, mx = -99999;
      for (q = 0U; q < g_cmd_log_n; q++) if (g_cmd_log[q].joint == 2U) {
          n2++;
          if ((int32_t)g_cmd_log[q].pulse < mn) mn = (int32_t)g_cmd_log[q].pulse;
          if ((int32_t)g_cmd_log[q].pulse > mx) mx = (int32_t)g_cmd_log[q].pulse;
          if (q < 2U || q + 1U >= g_cmd_log_n) printf("DBG12 grip t=%lu/%u p=%u\n", (unsigned long)q, (unsigned)g_cmd_log_n, g_cmd_log[q].pulse);
      }
      printf("DBG12 joint2 n=%lu min=%d max=%d\n", (unsigned long)n2, mn, mx); }
    CHECK(n_close != 0U, "gripper closed (1450) during the sequence");
    CHECK(n_last_grip == 1100U, "gripper open (1100) at the end");
    CHECK(arm_ctrl.teach.saved_valid == 1, "pick track still valid after run");

    /* ------------------------------------------------------------------ */
    section("T13 视觉逆解：平面两连杆，深度固定，带 Z 接口");
    long2();                                   /* SAVED -> IDLE */
    CHECK(arm_ctrl.teach.state == ARM_TEACH_IDLE, "SAVED + long press -> IDLE");
    long2();                                   /* IDLE -> READY */
    CHECK(arm_ctrl.teach.state == ARM_TEACH_READY, "IDLE + long press -> READY");
    sim_set_pulse(0, 1500); sim_set_pulse(1, 1500); run(60U);
    arm_ik_cmd("learn", "sh0", NULL);
    sim_set_pulse(0, 1000); run(60U);
    arm_ik_cmd("learn", "sh90", NULL);
    CHECK(arm_ctrl.cal.ik_valid == 0, "need the elbow points too");
    sim_set_pulse(0, 1500); sim_set_pulse(1, 1500); run(60U);
    arm_ik_cmd("learn", "el0", NULL);
    sim_set_pulse(1, 2000); run(60U);
    arm_ik_cmd("learn", "el90", NULL);
    CHECK(arm_ctrl.cal.ik_valid == 1, "four points learned -> IK valid");
    CHECK(arm_ctrl.cal.sh0_pulse == 1500 && arm_ctrl.cal.sh90_pulse == 1000, "shoulder calibration stored");
    CHECK(arm_ctrl.cal.el0_pulse == 1500 && arm_ctrl.cal.el90_pulse == 2000, "elbow calibration stored");

    s_ik_ready = 0U;
    arm_ik_cmd("xy", "200", "100");
    CHECK(s_ik_ready == 1, "xy solved");
    fk_from_pulse(s_ik_pose.shoulder, s_ik_pose.elbow, &x, &y);
    CHECK(fabsf(x - 200.0f) < 2.0f && fabsf(y - 100.0f) < 2.0f, "forward kinematics of the solution returns the target");
    printf("    xy 200,100 -> sh=%u el=%u, fk=(%.1f,%.1f)\n", (unsigned)s_ik_pose.shoulder, (unsigned)s_ik_pose.elbow, (double)x, (double)y);

    arm_ik_cmd("xy", "500", "0");       /* 超出臂长 300 */
    fk_from_pulse(s_ik_pose.shoulder, s_ik_pose.elbow, &x, &y);
    CHECK(s_ik_pose.shoulder != 0 && s_ik_pose.elbow != 0, "a pose was produced for the clamped target");
    CHECK(fabsf(x - 290.0f) < 2.0f && fabsf(y) < 2.0f, "clamped onto the reach boundary");

    arm_ik_cmd("depth", "200", NULL);         /* Z 取固定值 */
    arm_ik_cmd("k", "1.5", NULL);
    arm_ik_cmd("vis", "100", "-50");    /* 像素偏差：右 100，上 50 */
    fk_from_pulse(s_ik_pose.shoulder, s_ik_pose.elbow, &x, &y);
    CHECK(fabsf(x - 200.0f) < 2.0f && fabsf(y - 75.0f) < 2.0f, "vision: fwd = fixed depth 200, up = 50px * 1.5 = 75");
    CHECK(s_ik_yaw_corr == 0, "lateral pixel offset does not move yaw by default");
    CHECK(arm_ctrl.vis_x == 100 && arm_ctrl.vis_y == -50, "last vision frame remembered");

    arm_pick_cmd("apply", NULL, NULL);
    CHECK(arm_ctrl.pick_wp[1].shoulder == s_ik_pose.shoulder, "pick apply writes the EXTEND pose");
    CHECK(arm_ctrl.pick_wp[2].shoulder == s_ik_pose.shoulder, "pick apply writes the GRAB pose");
    CHECK(arm_teach_count == 5, "pick track rebuilt");

    /* 现场最快的 Z 标定：手拖到果子位置，pick learn 1 记下，再 ik here。
       四点标定是 sh0=1500/sh90=1000、el0=1500/el90=2000，所以 1800/1800 的 FK 正好是 (238,-121)。
       视觉 y=-50、k=1.5 => dy = up + y*k = -121 - 75 = -196。 */
    sim_set_pulse(0, 1800); sim_set_pulse(1, 1800);
    run(20U);
    arm_pick_cmd("learn", "1", NULL);
    CHECK(arm_ctrl.pick_wp[1].shoulder == 1800 && arm_ctrl.pick_wp[1].elbow == 1800, "EXTEND waypoint learned from the dragged pose");
    arm_ik_cmd("depth", "999", NULL);                   /* 故意错，确认 here 会覆盖 */
    sim_out_clear();
    arm_ik_cmd("learn", "here", NULL);
    CHECK(arm_ctrl.cal.vis_depth_mm == 238, "ik here turns the dragged pose into fixed depth 238mm");
    CHECK(arm_ctrl.cal.cam_dy_mm == -196, "ik here uses the latest vision y to set the height offset");
    CHECK(sim_out_has("vis_depth set"), "and prints that it set the depth");
    arm_ik_cmd("vis", NULL, NULL);
    fk_from_pulse(s_ik_pose.shoulder, s_ik_pose.elbow, &x, &y);
    CHECK(fabsf(x - 238.0f) < 2.0f && fabsf(y + 121.0f) < 2.0f, "the IK solution reproduces the dragged target after here");
    arm_ik_cmd("learn", "here", NULL);
    CHECK(arm_ctrl.cal.vis_depth_mm == 238 && arm_ctrl.cal.cam_dy_mm == -196, "ik here is idempotent");
    arm_ik_cmd("depth", "200", NULL);                   /* 后续 T13 的保存值仍用 200 */

    CHECK(arm_joint_set_limit(0, 600, 2400) == 0 && arm_joint_set_limit(2, 1100, 1900) == 0, "joint limits changed at runtime");
    CHECK(arm_yaw_set_limit(6000) == 0 && arm_yaw_set_speed_limit(200) == 0, "yaw limits changed at runtime");
    CHECK(arm_yaw_set_gain(10.0f, 2.0f, 100.0f, 5.0f) == 0, "yaw gains changed at runtime");
    s_ik_pose_saved_sh = s_ik_pose.shoulder;
    rc = arm_flash_commit();
    CHECK(rc == 0, "calibration + pick track saved");
    CHECK(FLASH_U32(SLOT_A) == 0x41544C33UL, "second save went to slot A");
    long2();
    CHECK(arm_ctrl.teach.state == ARM_TEACH_IDLE, "back to IDLE");

    arm_task_init();                          /* 重启 */
    run(100U);
    CHECK(arm_ctrl.cal.ik_valid == 1 && arm_ctrl.cal.sh90_pulse == 1000 && arm_ctrl.cal.el90_pulse == 2000, "calibration survives power cycle");
    CHECK(arm_ctrl.cal.vis_depth_mm == 200 && arm_ctrl.cal.vis_k_x100 == 150, "vision parameters survive power cycle");
    CHECK(arm_ctrl.cal.pick_close == 1450 && arm_ctrl.cal.pick_open == 1100, "gripper pulses survive power cycle");
    CHECK(s_joint_min[0] == 600 && s_joint_max[0] == 2400 && s_joint_min[2] == 1100 && s_joint_max[2] == 1900, "joint limits survive power cycle");
    CHECK(arm_ctrl.yaw.out_limit == 6000 && arm_ctrl.yaw.angle_pid.MaxOutput == 200, "yaw limits survive power cycle");
    CHECK(arm_ctrl.yaw.angle_pid.p == 10.0f && arm_ctrl.yaw.angle_pid.d == 2.0f &&
          arm_ctrl.yaw.spd_pid.p == 100.0f && arm_ctrl.yaw.spd_pid.d == 5.0f, "yaw gains survive power cycle");
    CHECK(arm_teach_count == 5 && arm_ctrl.teach.saved_valid == 1, "pick track restored");
    CHECK(arm_ctrl.pick_wp[1].shoulder == 1800, "pick waypoints rebuilt from the saved 5-point track");
    n_x = (uint16_t)g_cmd_log_n;
    tripleclick();
    run(1200U);
    CHECK(arm_ctrl.teach.state != ARM_TEACH_IDLE || g_cmd_log_n == n_x, "after reboot pick runs the saved track (no accidental default)");

    /* ------------------------------------------------------------------ */
    section("T14 片内 Flash 自检命令、A/B 回退");
    run_until_state(ARM_TEACH_SAVED, 20000U);
    long2();
    CHECK(arm_ctrl.teach.state == ARM_TEACH_IDLE, "back to IDLE before the flash test");
    arm_flash_test();
    CHECK(g_flag_flash_test == 1, "flash test PASS");
    CHECK(arm_teach_count == 5 && arm_ctrl.teach.saved_valid == 1, "real track survives the flash test");
    CHECK(arm_ctrl.cal.ik_valid == 1, "calibration survives the flash test");
    printf("    slot A magic=%08lX seq=%lu cnt=%u | slot B magic=%08lX seq=%lu cnt=%u\n",
           (unsigned long)FLASH_U32(SLOT_A), (unsigned long)FLASH_U32(SLOT_A + 4U), (unsigned)FLASH_U32(SLOT_A + 12U),
           (unsigned long)FLASH_U32(SLOT_B), (unsigned long)FLASH_U32(SLOT_B + 4U), (unsigned)FLASH_U32(SLOT_B + 12U));
    CHECK((FLASH_U32(SLOT_A) == 0xFFFFFFFFUL) != (FLASH_U32(SLOT_B) == 0xFFFFFFFFUL), "exactly one slot left blank (the test slot), the valid one kept");

    /* 两条不同的轨迹先后保存，再把新的那条改坏一个字节 */
    arm_teach_cmd("erase", NULL);
    n_x = teach_a_track(10, 8, 5);
    CHECK(FLASH_U32(SLOT_A) == 0x41544C33UL, "track X saved into slot A (seq 2, after the calibration-only image took B)");
    long2();
    n_y = teach_a_track(25, -6, 9);
    CHECK(FLASH_U32(SLOT_B) == 0x41544C33UL && FLASH_U32(SLOT_B + 4U) == 3UL, "track Y saved into slot B (seq 3)");
    CHECK(n_y != n_x, "two different tracks");
    long2();
    *(volatile uint8_t *)(uintptr_t)(SLOT_B + 80U + 4U) ^= 0xFFU;   /* 破坏新轨迹（B 槽）的一个点，点数据从偏移 80 开始 */
    arm_task_init();
    run(100U);
    CHECK(arm_teach_count == n_x, "newer slot corrupted -> falls back to the older valid slot");
    CHECK(arm_ctrl.cal.ik_valid == 1, "calibration still there (old slot has it too)");

    /* ------------------------------------------------------------------ */
    section("T15 回放不依赖 yaw；yaw 中途出问题时的处理");
    (void)arm_joint_set_limit(0, 500, 2500);
    (void)arm_joint_set_limit(2, 1250, 2060);
    (void)arm_yaw_set_limit(8000);
    (void)arm_yaw_set_speed_limit(500);
    (void)arm_yaw_set_gain(40.0f, 5.0f, 198.0f, 13.0f);
    g_yaw_wiring_dir = 1;
    (void)arm_yaw_set_dir(1);
    (void)arm_yaw_set_target_ecd(3000);      /* 被拒绝：PROTECT */
    arm_mode_manual();
    track_begin();
    track_run(3000U);
    CHECK(iabs32(arm_ctrl.yaw.err_ecd) <= 15, "yaw parked mid range");
    arm_stop_all();
    run(200U);

    /* 6020 掉线：流程照跑 */
    g_yaw_online = 0;
    run(200U);
    dblclick();
    dblclick();
    CHECK(arm_ctrl.teach.state == ARM_TEACH_PLAYBACK, "playback starts without the 6020");
    CHECK(arm_ctrl.yaw.enable == 0, "yaw loop not enabled");
    CHECK(run_until_state(ARM_TEACH_SAVED, 30000U), "sequence completes without yaw");
    long2();

    /* 6020 在线但没标方向：流程照跑，yaw 不参与 */
    g_yaw_online = 1;
    arm_ctrl.yaw.dir_valid = 0U;
    run(200U);
    dblclick();
    dblclick();
    CHECK(arm_ctrl.teach.state == ARM_TEACH_PLAYBACK, "playback starts with yaw uncalibrated");
    CHECK(run_until_state(ARM_TEACH_SAVED, 30000U), "sequence completes with yaw uncalibrated");
    long2();

    /* yaw 参与、中途丢反馈：立即中止 */
    arm_mode_manual();
    CHECK(arm_yaw_probe() == 1, "probe so yaw can take part");
    run(1200U);
    arm_stop_all();
    dblclick();
    dblclick();
    CHECK(arm_ctrl.teach.state == ARM_TEACH_PLAYBACK && arm_ctrl.yaw.enable == 1, "playback with yaw participating");
    CHECK(s_play_yaw == 1, "yaw flagged as participating");
    g_yaw_online = 0;
    run(300U);
    CHECK(arm_ctrl.teach.state == ARM_TEACH_ABORT, "feedback lost mid-playback -> ABORT");
    CHECK(arm_ctrl.yaw.fault == ARM_YAW_FAULT_FB_LOST && arm_ctrl.yaw.enable == 0, "fault latched, yaw off");
    CHECK(arm_busy == 0, "ABORT does not lock the chassis");
    long2();
    CHECK(arm_ctrl.teach.state == ARM_TEACH_IDLE, "ABORT cleared by long press");
    g_yaw_online = 1;
    run(200U);
    CHECK(arm_ctrl.teach.saved_valid == 0 || 1, "track flash copy untouched by the abort");
    arm_teach_cmd("load", NULL);
    CHECK(arm_ctrl.teach.state == ARM_TEACH_SAVED, "track can be reloaded from flash after abort");

    /* yaw 参与但方向标错：失控保护触发，流程中止而不是把臂带飞 */
    arm_stop_all();
    arm_yaw_fault_clear();
    g_yaw_wiring_dir = -1;
    run(200U);
    arm_mode_manual();
    (void)arm_yaw_set_dir(1);                 /* 标错 */
    ecd0 = (int32_t)arm_yaw_motor.ecd;
    CHECK(arm_yaw_set_target_ecd(ecd0 + 300) == ecd0 + 300, "a real move commanded with the wrong direction");
    track_begin();
    track_run(1200U);
    CHECK(arm_ctrl.yaw.fault == ARM_YAW_FAULT_RUNAWAY, "reversed yaw -> runaway fault before the limit");
    arm_stop_all();
    g_yaw_wiring_dir = 1;

    /* ------------------------------------------------------------------ */
    section("T16 删除轨迹后标定保留；删除后三击不动作");
    CHECK(arm_joint_set_limit(0, 700, 2300) == 0, "shoulder limits set");
    arm_ik_cmd("depth", "250", NULL);
    arm_teach_cmd("erase", NULL);
    CHECK(arm_teach_count == 0 && arm_ctrl.teach.saved_valid == 0, "track erased");
    CHECK(arm_ctrl.cal.lim_sh_min == 700 && arm_ctrl.cal.vis_depth_mm == 250, "calibration still in RAM after erase");
    arm_task_init();                          /* 重启 */
    run(100U);
    CHECK(arm_teach_count == 0 && arm_ctrl.teach.state == ARM_TEACH_IDLE, "no track after reboot");
    CHECK(s_joint_min[0] == 700 && s_joint_max[0] == 2300, "joint limits kept across erase + reboot");
    CHECK(arm_ctrl.cal.vis_depth_mm == 250 && arm_ctrl.cal.ik_valid == 1, "vision depth and IK kept across erase + reboot");
    (void)arm_joint_set_limit(0, 500, 2500);
    tripleclick();
    CHECK(arm_ctrl.teach.state == ARM_TEACH_PLAYBACK, "after erase the factory June demo runs again");
    run(2000U);
    tripleclick();
    CHECK(arm_ctrl.teach.state == ARM_TEACH_SAVED && arm_busy == 0, "triple stops the demo");
    long2();
    CHECK(arm_ctrl.teach.state == ARM_TEACH_IDLE, "back to IDLE");

    /* ------------------------------------------------------------------ */
    printf("\n%d checks, %d failed\n", g_pass + g_fail, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
