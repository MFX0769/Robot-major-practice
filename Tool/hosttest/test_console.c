/* 主机场景测试：真实的 Protocol/servo_test.c（控制台解析）、Application/chassis_task.c（底盘控制环）、
   Application/arm_task.c、Protocol/prot_rm_motor.c（电机怎么装进 CAN 帧）。
   串口寄存器、中断入口、CAN 发送队列用模拟的代替；底盘任务的死循环靠 osDelayUntil 的替身一圈圈放出来。
   跑法：py Tool\\hosttest\\run.py --console */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <setjmp.h>
#include <math.h>
#include "sim.h"
#include "main.h"
#include "arm_task.c"
#include "chassis_task.h"
#include "prot_rm_motor.h"
#include "prot_sbus.h"
#include "prot_8irtraking.h"
#include "servo_test.h"
#include "mode_switch_task.h"

static int g_fail, g_pass;
static const char *g_section = "";

#define CHECK(c, msg) do { if (c) { g_pass++; } else { g_fail++; printf("    FAIL [%s] %s (line %d)\n", g_section, msg, __LINE__); } } while (0)

static void section(const char *name)
{
    g_section = name;
    g_cmd_log_n = 0U;
    sim_out_clear();
    printf("== %s\n", name);
}

/* ---------------- 底盘任务：从死循环里一圈圈放出来 ----------------
   osDelayUntil 的替身：预算用完就把任务停在这一点、跳回测试；下一次调用再跳回去接着跑。
   任务第一次进入时垫 64KB 栈，停着的任务帧不会被测试代码自己的调用盖掉。 */
static jmp_buf s_test_ctx, s_task_ctx;
static volatile uint32_t s_budget;
static volatile int s_task_alive;
static int s_absmax;                 /* 这一轮里 4 个轮子电流绝对值的最大值 */
static int16_t s_last_tx[4];

static int chassis_hook(uint32_t *ticks, uint32_t period)
{
    int i, a;

    (void)ticks;
    for (i = 0; i < 4; i++) {
        s_last_tx[i] = chassis_2006[i].tx_current;
        a = (int)s_last_tx[i];
        if (a < 0) a = -a;
        if (a > s_absmax) s_absmax = a;
    }
    if (s_budget == 0U) {
        if (setjmp(s_task_ctx) == 0) longjmp(s_test_ctx, 1);
        return 0;
    }
    s_budget--;
    sim_advance(period);
    return 0;
}

static void chassis_enter(void)
{
    volatile char pad[65536];

    memset((void *)pad, 0, sizeof(pad));
    chassis_task(NULL);
}

static void chassis_run(uint32_t loops)              /* 让底盘任务正好再跑 loops 圈，每圈 2ms */
{
    if (loops == 0U) return;
    s_budget = loops - 1U;                            /* 第 loops 圈跑完后，钩子里的预算恰好见底 */
    s_absmax = 0;
    if (setjmp(s_test_ctx) == 0) {
        if (s_task_alive) longjmp(s_task_ctx, 1);
        chassis_enter();
    }
    s_task_alive = 1;
}

/* sgn = +1 四个轮子电流都为正，-1 都为负，0 都恰好是 0 */
static int all_sign(int sgn)
{
    int i;

    for (i = 0; i < 4; i++) {
        if (sgn > 0 && s_last_tx[i] <= 0) return 0;
        if (sgn < 0 && s_last_tx[i] >= 0) return 0;
        if (sgn == 0 && s_last_tx[i] != 0) return 0;
    }
    return 1;
}

/* 电流方向的组合。下标 0/1 = 左前/左后，2/3 = 右前/右后（右侧电机是反装的）。
   巡线往前走的方向是“左两轮正、右两轮负”；原地旋转是四个轮子同号。 */
static int pattern_line(void)       /* 和巡线前进同一个方向 */
{
    return s_last_tx[0] > 0 && s_last_tx[1] > 0 && s_last_tx[2] < 0 && s_last_tx[3] < 0;
}

static int pattern_reverse(void)    /* 和巡线前进相反 */
{
    return s_last_tx[0] < 0 && s_last_tx[1] < 0 && s_last_tx[2] > 0 && s_last_tx[3] > 0;
}

static void wheels_stop(void)
{
    int i;

    for (i = 0; i < 4; i++) sim_wheel_set_rpm(i, 0);
}

/* ---------------- 机械臂任务主循环：每圈 3ms，顺序与 arm_task() 一致 ---------------- */
static void arm_run(uint32_t ms)
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

static int wait_teach_state(arm_teach_state_e st, uint32_t timeout_ms)
{
    uint32_t end = g_now_ms + timeout_ms;

    while (g_now_ms < end) {
        arm_run(10U);
        if (arm_ctrl.teach.state == st) return 1;
    }
    return 0;
}

/* ---------------- 控制台输入：走串口接收模型和 USART10 中断入口 ---------------- */
static void cmd_feed(const char *line)           /* 只灌字节，不执行 */
{
    const char *p;

    for (p = line; *p != '\0'; p++) {
        sim_uart_rx_push(10, (uint8_t)*p);
        servo_test_rx_isr();
    }
}

static void cmd_drain(void)
{
    int i;

    for (i = 0; i < 12; i++) servo_test_process();    /* 机械臂任务每周期执行一条 */
}

/* 发一条命令：像终端一样以换行结尾。没有行尾的半行不会被固件执行 */
static void cmd(const char *line)
{
    cmd_feed(line);
    cmd_feed("\n");
    cmd_drain();
}

static const char *const s_bad[] = {
    " ", "joint", "joint shoulder", "joint shoulder 1550 extra", "joint knee 1500",
    "joint limit", "joint limit knee 1 2", "joint limit shoulder 1", "joint limit shoulder 1700 1400", "joint status x",
    "stop now", "teach", "teach bogus", "mode", "mode bogus",
    "flash", "flash test x", "flash bogus", "vis x", "w25", "w25 id x", "w25 format",
    "chassis", "chassis status x", "chassis line now", "chassis stop x", "chassis motor", "chassis motor 4 100",
    "chassis motor -1 100", "chassis motor 0", "chassis motor 0 0", "chassis motor 0 abc", "chassis motor 0 100 x",
    "chassis pid 9 1 2 3", "chassis pid 0 1 2", "chassis pid 0 1 2 3 4", "chassis pid 0 -1 2 3", "chassis pid 0 501 2 3",
    "chassis pid bogus",
    "can", "can stat x", "can recover", "can recover 3", "can recover 1 x", "can raw", "can raw 3 1", "can raw 1",
    "can raw 1 800", "can raw 1 xyz", "can raw 1 123 100", "can raw 1 123 zz",
    "uart x", "pick", "pick bogus", "pick learn", "pick learn 5", "pick learn x", "pick time 0 100", "pick time 5 100",
    "pick time 1 10", "pick time 1 20000", "pick open 100", "pick close 3000", "pick auto", "pick auto maybe",
    "pick vis 200 0", "pick vis 0 200", "ik", "ik bogus",
    "wp", "wp 5 1500 1500", "wp -1 1500 1500", "wp 1 1500", "wp 1 1500 1500 1500", "wp 1 100 1500", "wp 1 1500 2600",
    "wp x 1500 1500", "wp 1 x 1500",
    "yaw", "yaw bogus", "yaw status x", "yaw probe x", "yaw dir", "yaw dir 0", "yaw dir 2", "yaw limit", "yaw limit 0",
    "yaw speed", "yaw gain 1 2 3", "yaw gain 1 2 3 4 5", "yaw gain 1 2 3 x", "yaw fault x", "yaw step", "yaw step x",
    "yaw target", "yaw monitor x", "yaw stop x", "yaw set 1", "yaw clear x",
    "servo ping", "servo ping 300", "servo ping x", "servo read", "servo read x", "servo setid 2", "servo setid 2 2",
    "servo bind knee 2", "servo bind shoulder", "servo config x", "servo run x", "servo map x",
    "unknowncommand", "STOP", "mode\tmanual",
};

int main(int argc, char **argv)
{
    uint32_t n0, i, t0, n_cmd, lines;
    const uint8_t *d = g_canraw_d;
    dji_motor_t bad;
    int32_t ecd_target, n_close;

    sim_reset();
    g_verbose = (getenv("SIM_VERBOSE") != NULL);
    g_delayuntil_hook = chassis_hook;
    huart5.uart = 5;
    huart10.uart = 10;
    memset(&wbus_rc, 0, sizeof(wbus_rc));           /* 没插遥控器：通道和拨杆全是 0 */
    sim_line_set(LINE_STATE_OK, 0);
    arm_task_init();
    servo_test_init();
    g_capture = 1;

    /* 另起一个进程跑：上电后头 2 秒的巡线缓冲只在每次上电后出现一次 */
    if (argc > 1 && strcmp(argv[1], "boot") == 0) {
        section("C0 上电后头 2 秒进巡线：直行不看线，之后才看");
        sim_line_set(LINE_STATE_LOST, 40);           /* 一上来就丢线 */
        cmd("chassis line");
        cmd("chassis line");
        CHECK(arm_ctrl.chassis_run == 1, "line following started right after boot");
        chassis_run(900U);
        CHECK(pattern_line() && s_absmax > 0, "first 1.8s: straight at full speed, sensor ignored");
        chassis_run(300U);
        CHECK(all_sign(-1), "after 2s: line lost, last seen at +40 -> turn by memory (vx 350, wz -900)");
        chassis_run(400U);
        CHECK(all_sign(0), "line lost for more than 0.7s: stop");
        cmd("chassis stop");
        printf("\n%d checks, %d failed\n", g_pass + g_fail, g_fail);
        return (g_fail == 0) ? 0 : 1;
    }

    /* ------------------------------------------------------------------ */
    section("C1 上电：没有遥控器，保护模式下底盘发的全是零");
    CHECK(ctrl_mode == PROTECT_MODE && arm_busy == 0 && arm_ctrl.chassis_run == 0, "boot: PROTECT, nothing armed");
    CHECK(wbus_rc.data.sw2 == 0, "no remote: right switch reads 0");
    chassis_run(1000U);
    CHECK(g_chassis_tx_frames == 1000U, "one CAN frame per 2ms loop (500Hz)");
    CHECK(s_absmax == 0, "all four wheel currents stayed exactly 0 for 2s");
    CHECK(g_canraw_ch == 1U && g_canraw_id == 0x200U && g_canraw_n == 8U, "frame on CAN1, id 0x200, 8 bytes");
    CHECK(d[0] == 0U && d[1] == 0U && d[2] == 0U && d[3] == 0U && d[4] == 0U && d[5] == 0U && d[6] == 0U && d[7] == 0U,
          "frame is all zero");

    /* ------------------------------------------------------------------ */
    section("C2 电机怎么装进 CAN 帧（真实的 dji_send_current）");
    chassis_2006[0].tx_current = 1000;
    chassis_2006[1].tx_current = -1000;
    chassis_2006[2].tx_current = 300;
    chassis_2006[3].tx_current = -300;
    chassis_can_send();
    CHECK(g_canraw_ch == 1U && g_canraw_id == 0x200U, "chassis frame goes to CAN1 id 0x200");
    CHECK(d[0] == 0xFEU && d[1] == 0xD4U, "bytes 0-1 = motor[3] (ID1, right rear) = -300");
    CHECK(d[2] == 0xFCU && d[3] == 0x18U, "bytes 2-3 = motor[1] (ID2, left rear) = -1000");
    CHECK(d[4] == 0x01U && d[5] == 0x2CU, "bytes 4-5 = motor[2] (ID3, right front) = 300");
    CHECK(d[6] == 0x03U && d[7] == 0xE8U, "bytes 6-7 = motor[0] (ID4, left front) = 1000");
    CHECK(chassis_2006[0].can_id == 0x204U && chassis_2006[1].can_id == 0x202U &&
          chassis_2006[2].can_id == 0x203U && chassis_2006[3].can_id == 0x201U, "console index 0..3 = CAN ID 4/2/3/1");

    n0 = g_yaw_frames;
    arm_yaw_motor.tx_current = 1234;
    dji_send_current(&arm_yaw_motor, NULL, NULL, NULL);
    CHECK(g_canraw_ch == 2U && g_canraw_id == 0x1FFU && g_canraw_n == 8U, "yaw frame goes to CAN2 id 0x1FF");
    CHECK(d[6] == 0x04U && d[7] == 0xD2U, "bytes 6-7 = +1234 (ID4 slot)");
    CHECK(d[0] == 0U && d[1] == 0U && d[2] == 0U && d[3] == 0U && d[4] == 0U && d[5] == 0U, "bytes 0-5 stay 0");
    CHECK(g_yaw_tx == 1234 && g_yaw_frames == n0 + 1U, "the simulated 6020 is fed from exactly those bytes");
    arm_yaw_motor.tx_current = -25000;
    dji_send_current(&arm_yaw_motor, NULL, NULL, NULL);
    CHECK(d[6] == 0x9EU && d[7] == 0x58U && g_yaw_tx == -25000, "full negative voltage -25000 = 0x9E58");
    arm_yaw_motor.tx_current = 0;
    memset(&bad, 0, sizeof(bad));                  /* 从没注册过的电机：can_id 是 0 */
    n0 = g_canraw_cnt;
    dji_send_current(&bad, NULL, NULL, NULL);
    bad.can_id = 0x209U;
    dji_send_current(&bad, NULL, NULL, NULL);
    CHECK(g_canraw_cnt == n0, "a motor with an id outside 0x201..0x208 sends nothing (no write past the frame)");
    for (i = 0U; i < 4U; i++) chassis_2006[i].tx_current = 0;

    /* ------------------------------------------------------------------ */
    section("C3 mode manual：放开机械臂命令，底盘保持锁零；chassis line 要两条才启动");
    cmd("mode manual");
    CHECK(ctrl_mode == ARM_TEACH_MODE, "manual mode uses ARM_TEACH, not REMOTER");
    CHECK(arm_ctrl.manual == 1 && arm_busy == 1 && arm_ctrl.chassis_run == 0, "arm free, chassis locked");
    chassis_run(1500U);
    CHECK(s_absmax == 0, "with no remote the right switch reads 0 (= line-follow), still all zero while the arm owns the car");
    chassis_run(1U);
    CHECK(s_absmax == 0, "and no one-loop kick from a reference the PIDs saw while in PROTECT");
    cmd("chassis status");
    CHECK(sim_out_has("sw2=0 -> LINE_TRACK, arm_busy=1 jog=0"), "chassis status shows the truth");
    cmd("joint shoulder 1550");
    CHECK(g_cmd_log_n == 1U && g_cmd_log[0].joint == 0U && g_cmd_log[0].pulse == 1550U, "shoulder 1550 sent");
    cmd("yaw target 3000");
    CHECK(sim_out_has("rejected") && arm_ctrl.yaw.enable == 0, "yaw target needs 'yaw probe' first");
    cmd("chassis line");
    CHECK(sim_out_has("ARMED") && arm_ctrl.chassis_run == 0 && ctrl_mode == ARM_TEACH_MODE, "first 'chassis line' only arms");
    chassis_run(200U);
    CHECK(s_absmax == 0, "still zero while armed");
    cmd("chassis line");
    CHECK(sim_out_has("RUNNING") && arm_ctrl.chassis_run == 1 && ctrl_mode == REMOTER_MODE && arm_busy == 0,
          "second 'chassis line' within 5s starts line following");
    n_cmd = g_cmd_log_n;
    cmd("joint shoulder 1500");
    cmd("yaw enable");
    cmd("yaw probe");
    CHECK(g_cmd_log_n == n_cmd, "arm commands rejected while the chassis runs");
    cmd("mode manual");
    CHECK(arm_ctrl.chassis_run == 0 && arm_busy == 1 && ctrl_mode == ARM_TEACH_MODE, "'mode manual' takes the chassis back");
    wheels_stop();
    chassis_run(20U);
    CHECK(s_absmax == 0, "zero again");

    cmd("chassis line");
    cmd("chassis line");
    CHECK(arm_ctrl.chassis_run == 1, "started again");
    cmd("chassis stop");
    CHECK(ctrl_mode == PROTECT_MODE && arm_ctrl.chassis_run == 0 && arm_busy == 0 && sim_out_has("stopped, mode=PROTECT"),
          "'chassis stop' -> PROTECT");
    cmd("chassis line");
    chassis_run(3000U);                              /* 过 5 秒 */
    cmd("chassis line");
    CHECK(sim_out_has("ARMED") && arm_ctrl.chassis_run == 0, "the arming expires after 5s: has to be two again");
    cmd("chassis stop");
    cmd("chassis line");
    cmd("chassis line now");
    CHECK(sim_out_has("usage:") && arm_ctrl.chassis_run == 0, "'chassis line' with extra text is refused and cancels the arming");
    cmd("chassis line");
    cmd("chassis line");
    CHECK(arm_ctrl.chassis_run == 1, "started for the global stop test");
    cmd("stop");
    CHECK(ctrl_mode == PROTECT_MODE && arm_ctrl.chassis_run == 0, "global 'stop' also halts the chassis");

    /* ------------------------------------------------------------------ */
    section("C4 单轮开环点动：不用离开保护模式，1.5 秒自动归零");
    cmd("chassis motor 0 1000");
    CHECK(sim_out_has("open-loop") && chassis_dbg_jog_active() == 1, "accepted in PROTECT");
    chassis_run(10U);
    CHECK(s_last_tx[0] == 1000 && s_last_tx[1] == 0 && s_last_tx[2] == 0 && s_last_tx[3] == 0, "only wheel 0 gets 1000");
    CHECK(d[6] == 0x03U && d[7] == 0xE8U && d[0] == 0U && d[2] == 0U && d[4] == 0U, "and it lands in the ID4 slot of frame 0x200");
    chassis_run(800U);
    CHECK(chassis_dbg_jog_active() == 0, "timed out by itself");
    cmd("chassis motor 1 99999");
    chassis_run(2U);
    CHECK(s_last_tx[1] == 3000 && s_last_tx[0] == 0 && s_last_tx[2] == 0 && s_last_tx[3] == 0, "clamped to 3000, wheel 1 only");
    CHECK(d[2] == 0x0BU && d[3] == 0xB8U, "in the ID2 slot (bytes 2-3)");
    cmd("stop");
    wheels_stop();
    chassis_run(2U);
    CHECK(s_absmax == 0 && chassis_dbg_jog_active() == 0, "'stop' ends a jog at once");

    /* ------------------------------------------------------------------ */
    section("C5 接了遥控器：右拨杆上 = 摇杆直驱；左拨杆下或右拨杆中 = 零输出");
    wbus_rc.data.sw1 = 1;
    wbus_rc.data.sw2 = 2;
    wbus_rc.data.sw3 = 1;
    wbus_rc.data.ch1 = 0;
    wbus_rc.data.ch2 = 200;                          /* 右摇杆竖向：下 = 正 */
    cmd("chassis line");
    cmd("chassis line");
    CHECK(arm_ctrl.chassis_run == 1, "released");
    chassis_run(20U);
    CHECK(pattern_reverse(), "right stick down (ch2>0): vx=-600, opposite to the line-follow direction");
    wbus_rc.data.sw1 = 0;                            /* 左拨杆下 */
    chassis_run(1500U);                              /* 给定归零，轮子自己停下来 */
    CHECK(all_sign(0), "left switch down: zero output once the wheels have stopped");
    wbus_rc.data.sw1 = 1;
    wbus_rc.data.sw2 = 1;                            /* 右拨杆中 */
    chassis_run(1500U);
    CHECK(all_sign(0), "right switch middle: stop");
    wbus_rc.data.sw2 = 2;
    wbus_rc.data.ch2 = 0;
    wbus_rc.data.ch4 = -200;                         /* 左摇杆横：左 = 负 */
    chassis_run(60U);
    CHECK(all_sign(-1), "left stick left (ch4<0): all four wheels negative = spin");
    wbus_rc.data.ch4 = 200;
    chassis_run(2000U);                              /* 先刹住再反转 */
    CHECK(all_sign(1), "left stick right: the other way");
    cmd("mode manual");
    CHECK(arm_ctrl.chassis_run == 0 && arm_busy == 1, "manual takes it back even with a remote");
    chassis_run(2000U);
    CHECK(all_sign(0), "zero under manual once the wheels have stopped");
    cmd("chassis stop");
    memset(&wbus_rc, 0, sizeof(wbus_rc));            /* 拔掉遥控器 */
    wheels_stop();

    /* ------------------------------------------------------------------ */
    section("C6 巡线（不是上电后头 2 秒）：一启动就看线；丢线按记忆方向转 0.7 秒后停");
    sim_line_set(LINE_STATE_LOST, 40);               /* 一上来就丢线 */
    cmd("chassis line");
    cmd("chassis line");
    CHECK(arm_ctrl.chassis_run == 1, "line following started");
    chassis_run(300U);
    CHECK(all_sign(-1) && s_last_tx[2] < s_last_tx[0] && s_last_tx[3] < s_last_tx[1],
          "line lost, last seen at +40: vx 350 / wz -900 -> left -550, right -1250");
    chassis_run(400U);
    CHECK(all_sign(0), "line lost for more than 0.7s: stop (wheels have almost stopped, output exactly 0)");
    sim_line_set(LINE_STATE_OK, 0);
    chassis_run(100U);
    CHECK(pattern_line(), "line found again: straight (vx 1200)");
    sim_line_set(LINE_STATE_ALLON, 0);
    chassis_run(5U);
    CHECK(pattern_line(), "crossing (all sensors on): straight through");
    sim_line_set(LINE_STATE_OK, 0);
    cmd("chassis stop");
    CHECK(arm_ctrl.chassis_run == 0, "stopped");

    /* ------------------------------------------------------------------ */
    section("C7 参数不对的命令一律拒绝并打印用法，不产生任何动作");
    n_cmd = g_cmd_log_n;
    for (i = 0U; i < (uint32_t)(sizeof(s_bad) / sizeof(s_bad[0])); i++) {
        sim_out_clear();
        cmd(s_bad[i]);
        if (!sim_out_has("usage:")) {
            g_fail++;
            printf("    FAIL [%s] no usage printed for: '%s'\n", g_section, s_bad[i]);
        } else {
            g_pass++;
        }
    }
    cmd("joint shoulder abc");
    CHECK(sim_out_has("[JOG] rejected"), "a well-formed jog with a non-numeric value is refused");
    cmd("joint shoulder 1700");
    CHECK(sim_out_has("[JOG] rejected") && g_cmd_log_n == n_cmd, "a jog outside the 1400..1600 limits is refused");
    CHECK(g_cmd_log_n == n_cmd, "no servo command from any of them");
    CHECK(ctrl_mode == PROTECT_MODE && arm_ctrl.chassis_run == 0 && chassis_dbg_jog_active() == 0, "state untouched");

    /* ------------------------------------------------------------------ */
    section("C8 行组装：回车换行、首尾空格、超长行、队列满、USB 那一路");
    cmd("  mode manual  \r\n");
    CHECK(ctrl_mode == ARM_TEACH_MODE, "leading/trailing spaces and CRLF are fine");
    cmd_feed("mode prot");                           /* 一行分两次到 */
    cmd_feed("ect\n");
    cmd_drain();
    CHECK(ctrl_mode == PROTECT_MODE, "a line split across two arrivals is joined");
    cmd("stop\n\n\n\n");
    CHECK(sim_out_has("[ARM] stop"), "empty lines are ignored, the command runs once");
    lines = 0U;
    {
        char junk[80];
        memset(junk, 'a', sizeof(junk));
        junk[70] = '\0';
        cmd(junk);                                   /* 比 48 字节的行缓冲长 */
    }
    cmd("mode manual\n");
    CHECK(ctrl_mode == ARM_TEACH_MODE, "an overlong line is thrown away, the next command still works");
    cmd("mode protect\n");
    for (i = 0U; i < 12U; i++) {
        char line[32];
        snprintf(line, sizeof(line), "chassis motor 1 %lu\n", (unsigned long)(100U + i));
        cmd_feed(line);                              /* 12 条连着灌进来，没人执行 */
    }
    cmd_drain();
    chassis_run(2U);
    CHECK(s_last_tx[1] == 107, "queue is 8 deep: the first 8 ran, the last 4 were dropped (8th = 107)");
    cmd("stop");
    wheels_stop();
    {
        const char *u = "mode manual\n";
        for (i = 0U; i < strlen(u); i++) servo_test_rx_byte(u[i]);   /* USB 虚拟串口那一路 */
    }
    cmd_drain();
    CHECK(ctrl_mode == ARM_TEACH_MODE, "same parser over the USB CDC path");
    cmd("mode protect\n");

    /* ------------------------------------------------------------------ */
    section("C9 查询类命令：打印路径不崩，数字对得上");
    sim_dbus_set(111U, 7U, 1U);
    cmd("uart");
    CHECK(sim_out_has("dbus(U5) rx=111 frames=7 online=1") && sim_out_has("servo(U1) rx=1234 err=7"), "uart counters");
    cmd("vis");
    CHECK(sim_out_has("[VIS] bytes="), "vis");
    cmd("mode status");
    CHECK(sim_out_has("ctrl=PROTECT override=0 teach=IDLE"), "mode status");
    cmd("chassis status");
    CHECK(sim_out_has("m0(LF) id=0x204") && sim_out_has("m3(RB) id=0x201"), "chassis status lists id per wheel");
    sim_can_set(1, 500U, 0U, 248U, 0U, 12U);
    sim_can_set(2, 10U, 3U, 0U, 1U, 1U);
    cmd("can stat");
    CHECK(sim_out_has("c1 tx=500 rx=0 tec=248 rec=0 lec=0 bus_off_now=0 off_total=12"), "can stat CAN1 numbers");
    CHECK(sim_out_has("c2 tx=10 rx=3 tec=0 rec=0 lec=0 bus_off_now=1 off_total=1"), "can stat CAN2 numbers");
    CHECK(sim_out_has("rx stays 0 = nobody on the bus answers"), "the hint names the symptom");
    cmd("can recover 2");
    sim_out_clear();
    cmd("can stat");
    CHECK(sim_out_has("c2 tx=10 rx=3 tec=0 rec=0 lec=0 bus_off_now=0 off_total=1"), "'can recover' clears bus-off, counters kept");
    cmd("w25 id");
    CHECK(sim_out_has("(expect 0xEF4017)"), "w25 id");
    n0 = g_w25_erase_calls;
    cmd("w25 test");
    CHECK(sim_out_has("erase=0 write=0 match=1") && g_w25_erase_calls == n0 + 1U, "w25 test");
    cmd("servo ping 2");
    CHECK(sim_out_has("ping id=2 rc=0 reply="), "servo ping");
    cmd("servo read 2");
    {
        char want[48];
        snprintf(want, sizeof(want), "read id=2 rc=0 pulse=%u", (unsigned)g_pulse[0]);
        CHECK(sim_out_has(want), "servo read of a bound joint returns its position");
    }
    cmd("servo read 7");
    CHECK(sim_out_has("read id=7 rc=0 reply_id=7"), "servo read of an unbound id");
    cmd("servo map");
    CHECK(sim_out_has("map shoulder=2 elbow=1"), "servo map");
    cmd("servo setid 2 3");
    CHECK(sim_out_has("setid rejected") && xb_servo[0].id == 2U, "setid refused outside config mode");
    cmd("servo config");
    CHECK(servo_test_config_active() == 1U && g_config_active == 1U, "config mode on");
    cmd("servo setid 2 3");
    CHECK(sim_out_has("setid old=2 new=3 rc=0") && xb_servo[0].id == 5U, "setid executes in config mode");
    cmd("servo bind shoulder 2");
    CHECK(xb_servo[0].id == 2U, "bind restores the shoulder id");
    cmd("chassis line");
    cmd("chassis line");
    CHECK(sim_out_has("rejected: servo config active") && arm_ctrl.chassis_run == 0, "no line following in config mode");
    cmd("servo run");
    CHECK(servo_test_config_active() == 0U && g_config_active == 0U, "config mode off");
    cmd("joint status");
    CHECK(sim_out_has("shoulder pulse=") && sim_out_has("limits=1400..1600") && sim_out_has("gripper pulse="), "joint status");
    cmd("teach status");
    CHECK(sim_out_has("[TEACH] state=IDLE pts=0"), "teach status");
    cmd("pick status");
    CHECK(sim_out_has("0 READY") && sim_out_has("4 RELEASE"), "pick status lists the 5 steps");
    cmd("ik status");
    CHECK(sim_out_has("[IK]"), "ik status");
    cmd("yaw status");
    CHECK(sim_out_has("dir=1(uncalibrated)"), "yaw status");
    cmd("yaw fault");
    CHECK(sim_out_has("fault=0"), "yaw fault");
    cmd("yaw monitor");
    for (i = 0U; i < 20U; i++) servo_test_poll();
    CHECK(sim_out_has("[YAW_LIMIT] ecd="), "yaw monitor prints at 20Hz");
    cmd("yaw stop");
    cmd("chassis pid");
    CHECK(sim_out_has("pid m0 kp=5.00 ki=0.00 kd=1.00 max=8000"), "chassis pid defaults");
    cmd("chassis pid 0 1 2 3");
    CHECK(sim_out_has("pid motor set kp=1.00 ki=2.00 kd=3.00"), "chassis pid set");
    cmd("chassis pid line 4 0 0");
    CHECK(sim_out_has("pid line set kp=4.00"), "line pid set");

    /* ------------------------------------------------------------------ */
    section("C10 死程序按数字设路点（wp）：不读舵机也能跑");
    cmd("pick run");
    CHECK(sim_out_has("no saved track") && arm_ctrl.teach.state == ARM_TEACH_IDLE, "nothing taught yet: refuses to move");
    cmd("joint limit shoulder 500 2500");
    cmd("joint limit elbow 500 2500");
    CHECK(arm_ctrl.cal.lim_sh_min == 500 && arm_ctrl.cal.lim_el_max == 2500, "joint limits widened");
    cmd("wp 0 1500 1500");
    cmd("wp 1 1700 1900");
    cmd("wp 2 1700 1900");
    cmd("wp 3 1350 1300");
    cmd("wp 4 1500 1500");
    CHECK(arm_ctrl.pick_wp[1].shoulder == 1700 && arm_ctrl.pick_wp[1].elbow == 1900, "EXTEND waypoint set by numbers");
    CHECK(arm_teach_count == 5U && arm_ctrl.teach.saved_valid == 1U, "5-point track built");
    n_cmd = g_cmd_log_n;
    cmd("pick run");
    CHECK(arm_ctrl.teach.state == ARM_TEACH_PLAYBACK && arm_busy == 1, "pick run starts the sequence, chassis locked");
    cmd("wp 1 1500 1500");
    CHECK(sim_out_has("wp rejected while playing") && arm_ctrl.pick_wp[1].shoulder == 1700, "wp refused while playing");
    t0 = g_now_ms;
    CHECK(wait_teach_state(ARM_TEACH_SAVED, 20000U), "sequence finished");
    CHECK(g_cmd_log_n > n_cmd && g_gripper_cmd == 1500U, "servos were driven, gripper open at the end");
    n_close = 0;
    for (i = 0U; i < g_cmd_log_n; i++) if (g_cmd_log[i].joint == 2U && g_cmd_log[i].pulse == 1300U) n_close++;
    CHECK(n_close > 0, "gripper closed during the sequence");
    printf("    pick sequence took %lums\n", (unsigned long)(g_now_ms - t0));
    n0 = g_flash_erase_cnt;
    cmd("chassis line");
    cmd("chassis line");
    cmd("pick save");
    CHECK(sim_out_has("save rejected") && g_flash_erase_cnt == n0 && arm_ctrl.chassis_run == 1,
          "no flash write while the car is line-following (the CPU stalls during the write)");
    cmd("chassis stop");
    cmd("pick save");
    CHECK(sim_out_has("saved to internal flash") && g_flash_erase_cnt > n0, "pick save");
    g_servo_online[0] = 0;                           /* 舵机读不回来 */
    cmd("pick learn 1");
    CHECK(sim_out_has("learn failed: servo read") && arm_ctrl.pick_wp[1].shoulder == 1700, "pick learn fails cleanly");
    cmd("wp 1 1800 1800");
    CHECK(arm_ctrl.pick_wp[1].shoulder == 1800, "wp still works without reading the servos");
    g_servo_online[0] = 1;
    cmd("pick run");
    CHECK(arm_ctrl.teach.state == ARM_TEACH_PLAYBACK, "runs the edited track");
    cmd("pick run");
    CHECK(arm_ctrl.teach.state == ARM_TEACH_SAVED && arm_busy == 0, "'pick run' again stops it");
    cmd("stop");
    CHECK(arm_ctrl.teach.state == ARM_TEACH_IDLE && ctrl_mode == PROTECT_MODE, "back to IDLE / PROTECT");

    /* ------------------------------------------------------------------ */
    section("C11 yaw 走控制台：先探测方向，再给目标；回保护立即归零");
    cmd("mode manual");
    cmd("yaw enable");
    CHECK(sim_out_has("direction not calibrated") && arm_ctrl.yaw.enable == 0, "enable refused until probed");
    cmd("yaw probe");
    CHECK(sim_out_has("dir=1") && arm_ctrl.yaw.dir_valid == 1, "probe finds dir=+1");
    ecd_target = (int32_t)arm_yaw_motor.ecd + 300;
    {
        char line[32];
        snprintf(line, sizeof(line), "yaw target %ld\n", (long)ecd_target);
        cmd(line);
    }
    CHECK(arm_ctrl.yaw.enable == 1 && sim_out_has("[YAW] target"), "target accepted");
    arm_run(1500U);
    CHECK(iabs32(arm_ctrl.yaw.err_ecd) <= 15, "yaw reached the target given over the console");
    cmd("yaw status");
    CHECK(sim_out_has("en=1 on=1 flt=0 dir=1(ok)"), "yaw status");
    cmd("mode protect");
    CHECK(arm_ctrl.yaw.enable == 0 && g_yaw_tx == 0 && ctrl_mode == PROTECT_MODE, "leaving manual zeroes the yaw at once");

    /* ------------------------------------------------------------------ */
    section("C12 can raw：原样发一帧");
    cmd("can raw 1 123 01 02");
    CHECK(g_canraw_ch == 1U && g_canraw_id == 0x123U && g_canraw_n == 2U && d[0] == 1U && d[1] == 2U && sim_out_has("CAN1 id=0x123 len=2"),
          "CAN1 id 0x123 two bytes");
    cmd("can raw 2 7FF AA");
    CHECK(g_canraw_ch == 2U && g_canraw_id == 0x7FFU && g_canraw_n == 1U && d[0] == 0xAAU, "CAN2 id 0x7FF one byte");
    cmd("can raw 2 200");
    CHECK(g_canraw_ch == 2U && g_canraw_id == 0x200U && g_canraw_n == 0U, "zero data bytes is legal");

    printf("\n%d checks, %d failed\n", g_pass + g_fail, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
