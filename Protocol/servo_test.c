/**
  * @file servo_test.c
  * @brief 调试控制台：命令解析与执行
  * @note 输入有两路，内容相同：USART10（PE2/PE3，调试器虚拟串口）和板载 USB 虚拟串口。
  *       中断只收整行入队，机械臂任务上下文里逐条解析执行。
  */
#include "servo_test.h"
#include "prot_xb_servo.h"
#include "prot_servo.h"
#include "arm_yaw_limit.h"
#include "prot_rm_motor.h"
#include "control_def.h"
#include "arm_task.h"
#include "chassis_task.h"
#include "prot_vision.h"
#include "prot_8irtraking.h"
#include "prot_sbus.h"
#include "prot_dbus.h"
#include "w25q64.h"
#include "mode_switch_task.h"
#include "usart.h"
#include "dbg_print.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include <ctype.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static QueueHandle_t s_queue;
static char s_line[SERVO_TEST_LINE_MAX];
static uint8_t s_line_len;
static uint8_t s_config_allowed;
static uint8_t s_config_active;
static uint8_t s_yaw_monitor_active;
static uint8_t s_yaw_monitor_div;
static volatile uint32_t s_rx_bytes;
static uint32_t s_line_arm_tick;

static int32_t parse_signed(const char *s, int32_t *value)
{
    char *end = NULL;
    long v = strtol(s, &end, 10);
    if (s == end || *end != '\0' || v < INT32_MIN || v > INT32_MAX) {
        return 0;
    }
    *value = (int32_t)v;
    return 1;
}

static uint8_t parse_dec(const char *s, uint8_t *value)
{
    unsigned int v = 0;
    if (*s == '\0') return 0U;
    while (*s) {
        if (!isdigit((unsigned char)*s)) return 0U;
        v = v * 10U + (unsigned int)(*s - '0');
        if (v > 255U) return 0U;
        s++;
    }
    *value = (uint8_t)v;
    return 1U;
}

/* 小数点后最多按逐位累加解析，不依赖 strtof */
static uint8_t parse_float(const char *s, float *value)
{
    float sign = 1.0f, v = 0.0f, frac = 0.1f;
    uint8_t any = 0U;

    if (*s == '-') { sign = -1.0f; s++; }
    else if (*s == '+') s++;
    while (*s >= '0' && *s <= '9') {
        v = v * 10.0f + (float)(*s - '0');
        s++;
        any = 1U;
    }
    if (*s == '.') {
        s++;
        while (*s >= '0' && *s <= '9') {
            v += (float)(*s - '0') * frac;
            frac *= 0.1f;
            s++;
            any = 1U;
        }
    }
    if (*s != '\0' || any == 0U || v > 100000.0f) return 0U;
    *value = sign * v;
    return 1U;
}

static void servo_test_parse_line(const char *line)
{
    char work[SERVO_TEST_LINE_MAX];
    char *token;
    uint8_t id, old_id, new_id;
    uint8_t is_yaw_cmd;
    snprintf(work, sizeof(work), "%s", line);
    token = strtok(work, " ");
    if (token == NULL) goto usage;

    /* ---- 底盘：状态 / 免遥控器巡线 / 单电机点动 / PID ---- */
    if (strcmp(token, "chassis") == 0) {
        char *sub = strtok(NULL, " ");
        char *arg = strtok(NULL, " ");
        int32_t idx, cur;
        pid_t *p = NULL;
        float kp, ki, kd;
        char *t2, *t3;
        bsp_can_stat_t st;
        uint8_t i;
        int cm;

        if (sub == NULL) goto usage;
        if (strcmp(sub, "status") == 0) {
            if (arg != NULL) goto usage;
            cm = (wbus_rc.data.sw2 == 2) ? 1 : (wbus_rc.data.sw2 == 0) ? 2 : 0;
            bsp_can_stat_get(CAN_CHANNEL_1, &st);
            dbg_print("[CHASSIS] ctrl=%s sw2=%d -> %s, arm_busy=%u jog=%u | CAN1 tx=%lu rx=%lu tec=%u rec=%u bo=%u\r\n",
                      arm_mode_name(), (int)wbus_rc.data.sw2,
                      (cm == 2) ? "LINE_TRACK" : (cm == 1) ? "REMOTER" : "STOP",
                      (unsigned)arm_busy, (unsigned)chassis_dbg_jog_active(),
                      (unsigned long)st.tx_ok, (unsigned long)st.rx, st.tec, st.rec, st.bo);
            for (i = 0U; i < 4U; i++) {
                dbg_print("[CHASSIS] m%u(%s) id=0x%03lX on=%u ecd=%u rpm=%d cur=%d temp=%u\r\n", (unsigned)i,
                          (i == 0U) ? "LF" : (i == 1U) ? "LB" : (i == 2U) ? "RF" : "RB",
                          (unsigned long)chassis_2006[i].can_id, (unsigned)chassis_2006[i].online,
                          (unsigned)chassis_2006[i].ecd, (int)chassis_2006[i].speed_rpm,
                          (int)chassis_2006[i].tx_current, (unsigned)chassis_2006[i].temperature);
            }
            return;
        }
        if (strcmp(sub, "line") == 0) {
            /* 两步确认：巡线一启动就走，没有 2 秒缓冲（那只在上电后才有） */
            uint32_t now = HAL_GetTick();
            if (arg != NULL) goto usage;
            if (s_line_arm_tick == 0U || (uint32_t)(now - s_line_arm_tick) > 5000U) {
                s_line_arm_tick = now;
                dbg_print("[CHASSIS] line-follow ARMED, not running. Send 'chassis line' again within 5s to START (moves at once)\r\n");
                return;
            }
            s_line_arm_tick = 0U;
            if (arm_chassis_release() == 0) {
                mode_user_set(REMOTER_MODE);
                dbg_print("[CHASSIS] line-follow RUNNING (sw2=%d). 'chassis stop' or 'stop' to halt\r\n",
                          (int)wbus_rc.data.sw2);
            }
            return;
        }
        if (strcmp(sub, "stop") == 0) {
            if (arg != NULL) goto usage;
            s_line_arm_tick = 0U;
            chassis_dbg_jog_stop();
            arm_chassis_stop();
            dbg_print("[CHASSIS] stopped, mode=PROTECT\r\n");
            return;
        }
        if (strcmp(sub, "motor") == 0) {
            if (arg == NULL || !parse_signed(arg, &idx) || idx < 0 || idx > 3) goto usage;
            arg = strtok(NULL, " ");
            if (arg == NULL || !parse_signed(arg, &cur) || strtok(NULL, " ") != NULL) goto usage;
            if (chassis_dbg_jog_start(idx, cur) != 0) goto usage;
            dbg_print("[CHASSIS] motor %ld open-loop, clamped to +-%d, auto zero after %ums\r\n",
                      (long)idx, CHASSIS_JOG_MAX_CUR, (unsigned)CHASSIS_JOG_MS);
            return;
        }
        if (strcmp(sub, "map") == 0) {
            char *t2 = strtok(NULL, " ");
            int32_t sg;

            if (arg == NULL && t2 == NULL) {
                dbg_print("[CHASSIS] map vx=%+d vy=%+d wz=%+d ('chassis map vx|vy|wz <1|-1>', 'pick save' keeps it)\r\n",
                          (int)g_chassis_map_sign[0], (int)g_chassis_map_sign[1], (int)g_chassis_map_sign[2]);
                return;
            }
            if (arg == NULL || t2 == NULL || strtok(NULL, " ") != NULL) goto usage;
            if (strcmp(arg, "vx") == 0) idx = 0;
            else if (strcmp(arg, "vy") == 0) idx = 1;
            else if (strcmp(arg, "wz") == 0) idx = 2;
            else goto usage;
            if (!parse_signed(t2, &sg) || arm_chassis_map_set((uint8_t)idx, sg) != 0) goto usage;
            return;
        }
        if (strcmp(sub, "pid") == 0) {
            if (arg == NULL) {
                for (i = 0U; i < 4U; i++) {
                    p = chassis_dbg_pid_get(i);
                    dbg_print("[CHASSIS] pid m%u kp=%.2f ki=%.2f kd=%.2f max=%lu\r\n", (unsigned)i,
                              (double)p->p, (double)p->i, (double)p->d, (unsigned long)p->MaxOutput);
                }
                p = chassis_dbg_line_pid_get();
                dbg_print("[CHASSIS] pid line kp=%.2f ki=%.2f kd=%.2f max=%lu\r\n",
                          (double)p->p, (double)p->i, (double)p->d, (unsigned long)p->MaxOutput);
                return;
            }
            if (strcmp(arg, "line") == 0) {
                p = chassis_dbg_line_pid_get();
            } else if (parse_signed(arg, &idx) && idx >= 0 && idx <= 3) {
                p = chassis_dbg_pid_get((uint8_t)idx);
            } else goto usage;
            t2 = strtok(NULL, " ");
            t3 = strtok(NULL, " ");
            arg = strtok(NULL, " ");
            if (t2 == NULL || t3 == NULL || arg == NULL || strtok(NULL, " ") != NULL) goto usage;
            if (!parse_float(t2, &kp) || !parse_float(t3, &ki) || !parse_float(arg, &kd)) goto usage;
            if (kp < 0.0f || kp > 500.0f || ki < 0.0f || ki > 500.0f || kd < 0.0f || kd > 500.0f) goto usage;
            p->p = kp;
            p->i = ki;
            p->d = kd;
            dbg_print("[CHASSIS] pid %s set kp=%.2f ki=%.2f kd=%.2f\r\n",
                      (p == chassis_dbg_line_pid_get()) ? "line" : "motor", (double)kp, (double)ki, (double)kd);
            return;
        }
        goto usage;
    }

    /* ---- CAN 总线：统计 / 手动恢复 / 原始帧 ---- */
    if (strcmp(token, "can") == 0) {
        char *sub = strtok(NULL, " ");
        char *arg;
        char *end = NULL;
        int32_t ch;
        uint32_t id;
        uint8_t d[8] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
        uint8_t n = 0U;
        bsp_can_stat_t st;
        uint8_t k;

        if (sub == NULL) goto usage;
        if (strcmp(sub, "stat") == 0) {
            if (strtok(NULL, " ") != NULL) goto usage;
            for (k = 0U; k < 2U; k++) {
                bsp_can_stat_get((k == 0U) ? CAN_CHANNEL_1 : CAN_CHANNEL_2, &st);
                dbg_print("[CAN] c%u tx=%lu rx=%lu tec=%u rec=%u lec=%u bus_off_now=%u off_total=%lu auto_recovered=%lu\r\n",
                          (unsigned)(k + 1U), (unsigned long)st.tx_ok, (unsigned long)st.rx,
                          st.tec, st.rec, st.lec, st.bo,
                          (unsigned long)st.bus_off, (unsigned long)st.recovered);
            }
            dbg_print("[CAN] readings: rx stays 0 = nobody on the bus answers (esc power / wiring / terminator / wrong bus). "
                      "off_total growing = no ACK; the bus goes off and comes back by itself, it recovers once a node answers. "
                      "bus_off_now=1 for long = send 'can recover <1|2>'. tx counts frames handed to the peripheral, it grows even with nobody on the bus\r\n");
            return;
        }
        if (strcmp(sub, "recover") == 0) {
            arg = strtok(NULL, " ");
            if (arg == NULL || !parse_signed(arg, &ch) || (ch != 1 && ch != 2) || strtok(NULL, " ") != NULL) goto usage;
            bsp_can_recover((ch == 1) ? CAN_CHANNEL_1 : CAN_CHANNEL_2);
            dbg_print("[CAN] CAN%ld re-initialized, filters kept\r\n", (long)ch);
            return;
        }
        if (strcmp(sub, "raw") == 0) {
            arg = strtok(NULL, " ");
            if (arg == NULL || !parse_signed(arg, &ch) || (ch != 1 && ch != 2)) goto usage;
            arg = strtok(NULL, " ");
            if (arg == NULL) goto usage;
            id = strtoul(arg, &end, 16);
            if (end == arg || *end != '\0' || id > 0x7FFU) goto usage;
            while ((arg = strtok(NULL, " ")) != NULL && n < 8U) {
                unsigned long b = strtoul(arg, &end, 16);
                if (end == arg || *end != '\0' || b > 255UL) goto usage;
                d[n++] = (uint8_t)b;
            }
            if (strtok(NULL, " ") != NULL) goto usage;
            if (ch == 1) Prot_CAN1_Send_Frame(id, d, n);
            else Prot_CAN2_Send_Frame(id, d, n);
            dbg_print("[CAN] CAN%ld id=0x%03lX len=%u queued\r\n", (long)ch, (unsigned long)id, (unsigned)n);
            return;
        }
        goto usage;
    }

    /* ---- 各路串口接收计数：哪一路不收数一眼能看出来 ---- */
    if (strcmp(token, "uart") == 0) {
        const prot_8irtraking_data_t *ln;
        if (strtok(NULL, " ") != NULL) goto usage;
        ln = prot_8irtraking_get_data();
        dbg_print("[UART] console(U10) rx=%lu | line(U7) rx=%lu frames=%lu | vision(U8/PE0) rx=%lu frames=%lu\r\n",
                  (unsigned long)s_rx_bytes, (unsigned long)ir_isr_bytes,
                  (unsigned long)ln->rx_cnt, (unsigned long)vision_uart_rx_bytes,
                  (unsigned long)vision_uart_rx_frames);
        dbg_print("[UART] dbus(U5) rx=%lu frames=%lu online=%u | servo(U1) rx=%lu err=%lu\r\n",
                  (unsigned long)dbus_rx_bytes(), (unsigned long)dbus_rx_count(),
                  (unsigned)dbus_online(), (unsigned long)xb_servo_rx_bytes(),
                  (unsigned long)xb_servo_rx_errors());
        return;
    }

    /* ---- 遥控器原始值：调方向第一步就是它 ---- */
    if (strcmp(token, "rc") == 0) {
        if (strtok(NULL, " ") != NULL) goto usage;
        dbg_print("[RC] ch1=%d ch2=%d ch3=%d ch4=%d | sw1=%u sw2=%u sw3=%u | lock=%u dbus=%u rx=%lu\r\n",
                  (int)wbus_rc.data.ch1, (int)wbus_rc.data.ch2, (int)wbus_rc.data.ch3, (int)wbus_rc.data.ch4,
                  (unsigned)wbus_rc.data.sw1, (unsigned)wbus_rc.data.sw2, (unsigned)wbus_rc.data.sw3,
                  (unsigned)lock_flag, (unsigned)dbus_online(), (unsigned long)dbus_rx_count());
        dbg_print("[RC] right stick ch2=fwd ch1=side, left stick ch3=spin; sw2 down + sw1 not mid = line follow\r\n");
        return;
    }

    /* ---- 死程序采摘 / 视觉逆解 ---- */
    if (strcmp(token, "pick") == 0 || strcmp(token, "ik") == 0) {
        uint8_t is_pick = (token[0] == 'p') ? 1U : 0U;
        char *sub = strtok(NULL, " ");
        char *a1 = strtok(NULL, " ");
        char *a2 = strtok(NULL, " ");
        if (sub == NULL || strtok(NULL, " ") != NULL) goto usage;
        if (is_pick != 0U) arm_pick_cmd(sub, a1, a2);
        else arm_ik_cmd(sub, a1, a2);
        return;
    }

    /* ---- 按数字设采摘路点（不读舵机）：wp <0..4> <sh> <el> ---- */
    if (strcmp(token, "wp") == 0) {
        int32_t n, sh, el;
        char *a1 = strtok(NULL, " ");
        char *a2 = strtok(NULL, " ");
        char *a3 = strtok(NULL, " ");

        if (a1 == NULL || a2 == NULL || a3 == NULL || strtok(NULL, " ") != NULL) goto usage;
        if (!parse_signed(a1, &n) || n < 0 || n >= (int32_t)ARM_PICK_STEPS) goto usage;
        if (!parse_signed(a2, &sh) || !parse_signed(a3, &el)) goto usage;
        if (sh < (int32_t)SERVO_PULSE_MIN || sh > (int32_t)SERVO_PULSE_MAX ||
            el < (int32_t)SERVO_PULSE_MIN || el > (int32_t)SERVO_PULSE_MAX) goto usage;
        (void)arm_pick_set((uint8_t)n, (uint16_t)sh, (uint16_t)el);
        return;
    }

    /* ---- 四关节点动 / 全局停止 ---- */
    if (strcmp(token, "joint") == 0) {
        uint8_t joint;
        int rc;
        token = strtok(NULL, " ");
        if (token == NULL) goto usage;
        if (strcmp(token, "status") == 0) {
            if (strtok(NULL, " ") != NULL) goto usage;
            arm_joint_status();
            return;
        }
        if (strcmp(token, "limit") == 0) {
            int32_t lo, hi;
            token = strtok(NULL, " ");
            if (token == NULL) goto usage;
            if (strcmp(token, "shoulder") == 0) joint = 0U;
            else if (strcmp(token, "elbow") == 0) joint = 1U;
            else if (strcmp(token, "gripper") == 0) joint = 2U;
            else goto usage;
            token = strtok(NULL, " ");
            if (token == NULL || !parse_signed(token, &lo)) goto usage;
            token = strtok(NULL, " ");
            if (token == NULL || !parse_signed(token, &hi) || strtok(NULL, " ") != NULL) goto usage;
            if (arm_joint_set_limit(joint, lo, hi) != 0) goto usage;
            return;
        }
        if (strcmp(token, "shoulder") == 0) joint = 0U;
        else if (strcmp(token, "elbow") == 0) joint = 1U;
        else if (strcmp(token, "gripper") == 0) joint = 2U;
        else goto usage;
        token = strtok(NULL, " ");
        if (token == NULL || strtok(NULL, " ") != NULL) goto usage;
        rc = arm_joint_jog(joint, token);
        if (rc != 0) dbg_print("[JOG] rejected rc=%d\r\n", rc);
        return;
    }
    if (strcmp(token, "stop") == 0) {
        if (strtok(NULL, " ") != NULL) goto usage;
        chassis_dbg_jog_stop();
        arm_stop_all();
        return;
    }

    /* ---- 示教状态机 ---- */
    if (strcmp(token, "teach") == 0) {
        char *sub = strtok(NULL, " ");
        char *arg = strtok(NULL, " ");
        if (sub == NULL || strtok(NULL, " ") != NULL) goto usage;
        arm_teach_cmd(sub, arg);
        return;
    }

    /* ---- 模式覆盖 ---- */
    if (strcmp(token, "mode") == 0) {
        token = strtok(NULL, " ");
        if (token == NULL || strtok(NULL, " ") != NULL) goto usage;
        if (strcmp(token, "manual") == 0) {
            (void)arm_mode_manual();
        } else if (strcmp(token, "protect") == 0) {
            chassis_dbg_jog_stop();
            arm_stop_all();
        } else if (strcmp(token, "status") == 0) {
            dbg_print("[MODE] ctrl=%s override=%u teach=%s\r\n", arm_mode_name(),
                      (unsigned)mode_user_active(), arm_teach_state_name());
        } else goto usage;
        return;
    }

    /* ---- 片内 Flash 验证（不需要电机供电） ---- */
    if (strcmp(token, "flash") == 0) {
        token = strtok(NULL, " ");
        if (token == NULL || strcmp(token, "test") != 0 || strtok(NULL, " ") != NULL) goto usage;
        arm_flash_test();
        return;
    }

    /* ---- 视觉链路状态 ---- */
    if (strcmp(token, "vis") == 0) {
        uint32_t age = 0U;
        if (strtok(NULL, " ") != NULL) goto usage;
        if (vision_uart_last_tick != 0U) age = HAL_GetTick() - vision_uart_last_tick;
        dbg_print("[VIS] bytes=%lu frames=%lu age=%lu x=%d y=%d flag=%u cnt=%lu\r\n",
                  (unsigned long)vision_uart_rx_bytes,
                  (unsigned long)vision_uart_rx_frames,
                  (unsigned long)age,
                  (int)vision_fruit.x_dis, (int)vision_fruit.y_dis,
                  (unsigned)vision_fruit.flag,
                  (unsigned long)vision_fruit.rec_cnt);
        return;
    }

    /* ---- W25Q64 ---- */
    if (strcmp(token, "w25") == 0) {
        token = strtok(NULL, " ");
        if (token == NULL || strtok(NULL, " ") != NULL) goto usage;
        if (strcmp(token, "id") == 0) {
            dbg_print("[W25] id=0x%08lX (expect 0xEF4017)\r\n",
                      (unsigned long)OSPI_W25Qxx_ReadID());
        } else if (strcmp(token, "test") == 0) {
            uint8_t wbuf[16], rbuf[16], i, ok = 1U;
            int8_t rc1, rc2;
            for (i = 0U; i < 16U; i++) wbuf[i] = (uint8_t)(0x5AU + i * 7U);
            rc1 = OSPI_W25Qxx_BlockErase_32K(0x7E0000UL);
            rc2 = OSPI_W25Qxx_WriteBuffer(wbuf, 0x7E0000UL, 16U);
            (void)OSPI_W25Qxx_ReadBuffer(rbuf, 0x7E0000UL, 16U);
            for (i = 0U; i < 16U; i++) if (wbuf[i] != rbuf[i]) ok = 0U;
            dbg_print("[W25] test erase=%d write=%d match=%u\r\n",
                      rc1, rc2, (unsigned)ok);
        } else goto usage;
        return;
    }

    is_yaw_cmd = (strcmp(token, "yaw") == 0) ? 1U : 0U;
    if (!is_yaw_cmd && strcmp(token, "servo") != 0) goto usage;
    token = strtok(NULL, " ");
    if (token == NULL) goto usage;

    if (is_yaw_cmd) {
        if (strcmp(token, "enable") == 0) {
            (void)arm_yaw_set_enable(1U);
        } else if (strcmp(token, "disable") == 0) {
            (void)arm_yaw_set_enable(0U);
        } else if (strcmp(token, "status") == 0) {
            if (strtok(NULL, " ") != NULL) goto usage;
            dbg_print("[YAW] en=%u on=%u flt=%u dir=%d(%s) tgt=%ld fdb=%ld err=%ld spd=%d/%.0f out=%.0f lim=%d\r\n",
                      (unsigned)arm_ctrl.yaw.enable, (unsigned)arm_ctrl.yaw.online,
                      (unsigned)arm_ctrl.yaw.fault, (int)arm_ctrl.yaw.dir,
                      (arm_ctrl.yaw.dir_valid != 0U) ? "ok" : "uncalibrated",
                      (long)arm_ctrl.yaw.target_ecd, (long)arm_ctrl.yaw.fdb_ecd,
                      (long)arm_ctrl.yaw.err_ecd,
                      (int)arm_ctrl.yaw.spd_fdb, arm_ctrl.yaw.spd_ref,
                      arm_ctrl.yaw.out, (int)arm_ctrl.yaw.out_limit);
            dbg_print("[YAW] gain pos kp=%.1f kd=%.1f, spd kp=%.1f kd=%.1f, spd max=%lu rpm\r\n",
                      (double)arm_ctrl.yaw.angle_pid.p, (double)arm_ctrl.yaw.angle_pid.d,
                      (double)arm_ctrl.yaw.spd_pid.p, (double)arm_ctrl.yaw.spd_pid.d,
                      (unsigned long)arm_ctrl.yaw.angle_pid.MaxOutput);
        } else if (strcmp(token, "probe") == 0) {
            if (strtok(NULL, " ") != NULL) goto usage;
            (void)arm_yaw_probe();
        } else if (strcmp(token, "dir") == 0) {
            int32_t d;
            token = strtok(NULL, " ");
            if (token == NULL || !parse_signed(token, &d) || strtok(NULL, " ") != NULL) goto usage;
            if (arm_yaw_set_dir(d) != 0) goto usage;
        } else if (strcmp(token, "limit") == 0) {
            int32_t n;
            token = strtok(NULL, " ");
            if (token == NULL || !parse_signed(token, &n) || strtok(NULL, " ") != NULL) goto usage;
            if (arm_yaw_set_limit(n) != 0) goto usage;
        } else if (strcmp(token, "speed") == 0) {
            int32_t n;
            token = strtok(NULL, " ");
            if (token == NULL || !parse_signed(token, &n) || strtok(NULL, " ") != NULL) goto usage;
            if (arm_yaw_set_speed_limit(n) != 0) goto usage;
        } else if (strcmp(token, "gain") == 0) {
            float kp_pos, kd_pos, kp_spd, kd_spd;
            char *t1 = strtok(NULL, " ");
            char *t2 = strtok(NULL, " ");
            char *t3 = strtok(NULL, " ");
            char *t4 = strtok(NULL, " ");
            if (t1 == NULL || t2 == NULL || t3 == NULL || t4 == NULL || strtok(NULL, " ") != NULL) goto usage;
            if (!parse_float(t1, &kp_pos) || !parse_float(t2, &kd_pos) ||
                !parse_float(t3, &kp_spd) || !parse_float(t4, &kd_spd)) goto usage;
            if (arm_yaw_set_gain(kp_pos, kd_pos, kp_spd, kd_spd) != 0) goto usage;
        } else if (strcmp(token, "fault") == 0) {
            token = strtok(NULL, " ");
            if (token == NULL) {
                dbg_print("[YAW] fault=%u (0=none, 1=runaway, 2=feedback lost in playback)\r\n",
                          (unsigned)arm_ctrl.yaw.fault);
            } else if (strcmp(token, "clear") == 0 && strtok(NULL, " ") == NULL) {
                arm_yaw_fault_clear();
            } else goto usage;
        } else if (strcmp(token, "step") == 0) {
            int32_t delta;
            int rc = 0;
            token = strtok(NULL, " ");
            if (token == NULL || !parse_signed(token, &delta) || strtok(NULL, " ") != NULL) goto usage;
            rc = arm_yaw_set_target_ecd((int32_t)arm_yaw_motor.ecd + delta);
            if (rc < 0) dbg_print("[YAW] step rejected rc=%d\r\n", rc);
            else dbg_print("[YAW] step %ld -> %d\r\n", (long)delta, rc);
        } else if (strcmp(token, "target") == 0) {
            int32_t ecd;
            int rc = 0;
            token = strtok(NULL, " ");
            if (token == NULL || !parse_signed(token, &ecd) || strtok(NULL, " ") != NULL) goto usage;
            rc = arm_yaw_set_target_ecd(ecd);
            if (rc < 0) dbg_print("[YAW] target rejected rc=%d\r\n", rc);
            else dbg_print("[YAW] target %ld -> %d\r\n", (long)ecd, rc);
        } else if (strcmp(token, "monitor") == 0) {
            if (strtok(NULL, " ") != NULL) goto usage;
            s_yaw_monitor_active = 1U;
            s_yaw_monitor_div = 0U;
            dbg_print("[YAW_LIMIT] monitor ON: zero torque, 20Hz\r\n");
        } else if (strcmp(token, "stop") == 0) {
            if (strtok(NULL, " ") != NULL) goto usage;
            s_yaw_monitor_active = 0U;
            dbg_print("[YAW_LIMIT] monitor OFF\r\n");
        } else if (strcmp(token, "set") == 0) {
            int32_t lower, upper;
            token = strtok(NULL, " ");
            if (token == NULL || !parse_signed(token, &lower)) goto usage;
            token = strtok(NULL, " ");
            if (token == NULL || !parse_signed(token, &upper)) goto usage;
            if (strtok(NULL, " ") != NULL) goto usage;
            if (arm_yaw_limit_set(lower, upper) != 0) {
                dbg_print("[YAW_LIMIT] set rejected: lower must be less than upper\r\n");
            } else {
                dbg_print("[YAW_LIMIT] set lower_ecd=%ld upper_ecd=%ld\r\n",
                          (long)arm_yaw_limit_config.lower_ecd,
                          (long)arm_yaw_limit_config.upper_ecd);
            }
        } else if (strcmp(token, "clear") == 0) {
            if (strtok(NULL, " ") != NULL) goto usage;
            arm_yaw_limit_clear();
            dbg_print("[YAW_LIMIT] limits cleared\r\n");
        } else {
            goto usage;
        }
        return;
    }

    if (strcmp(token, "ping") == 0) {
        token = strtok(NULL, " ");
        if (token == NULL || !parse_dec(token, &id) || id > 254U || strtok(NULL, " ") != NULL) goto usage;
        uint8_t rc = xb_servo_ping_raw(id);
        dbg_print("[SERVO_TEST] ping id=%u rc=%u reply=%s\r\n", id, rc, xb_servo_last_frame());
    } else if (strcmp(token, "read") == 0) {
        token = strtok(NULL, " ");
        if (token == NULL || !parse_dec(token, &id) || id > 254U || strtok(NULL, " ") != NULL) goto usage;
        uint16_t pulse = 0;
        uint8_t found = 0;
        for (uint8_t j = 0; j < XB_JOINT_NUM; j++) {
            if (xb_servo[j].id == id) {
                uint8_t rc = xb_servo_read_pos(j, &pulse);
                dbg_print("[SERVO_TEST] read id=%u rc=%u pulse=%u\r\n", id, rc, pulse);
                found = 1U;
                break;
            }
        }
        if (!found) {
            uint8_t read_id = 0;
            uint8_t rc = xb_servo_read_id_raw(id, &read_id);
            dbg_print("[SERVO_TEST] read id=%u rc=%u reply_id=%u\r\n", id, rc, read_id);
        }
    } else if (strcmp(token, "setid") == 0) {
        token = strtok(NULL, " ");
        if (token == NULL || !parse_dec(token, &old_id)) goto usage;
        token = strtok(NULL, " ");
        if (token == NULL || !parse_dec(token, &new_id)) goto usage;
        if (old_id > 254U || new_id > 254U || old_id == new_id || strtok(NULL, " ") != NULL) goto usage;
        if (!s_config_allowed || !s_config_active) {
            dbg_print("[SERVO_TEST] setid rejected: send 'servo config' first\r\n");
            return;
        }
        uint8_t rc = xb_servo_write_id_raw(old_id, new_id);
        dbg_print("[SERVO_TEST] setid old=%u new=%u rc=%u reply=%s\r\n",
                  old_id, new_id, rc, xb_servo_last_frame());
    } else if (strcmp(token, "bind") == 0) {
        uint8_t joint = 0;
        token = strtok(NULL, " ");
        if (token == NULL) goto usage;
        if (strcmp(token, "shoulder") == 0) joint = XB_SHOULDER;
        else if (strcmp(token, "elbow") == 0) joint = XB_ELBOW;
        else goto usage;
        token = strtok(NULL, " ");
        if (token == NULL || !parse_dec(token, &id) || id > 254U || strtok(NULL, " ") != NULL) goto usage;
        uint8_t rc = xb_servo_bind(joint, id);
        dbg_print("[SERVO_TEST] bind joint=%u id=%u rc=%u\r\n", joint, id, rc);
    } else if (strcmp(token, "config") == 0) {
        if (strtok(NULL, " ") != NULL) goto usage;
        servo_set_config_mode(1);
        s_config_active = 1;
        dbg_print("[SERVO_TEST] config mode ON: power held, motion inhibited, setid enabled\r\n");
    } else if (strcmp(token, "run") == 0) {
        if (strtok(NULL, " ") != NULL) goto usage;
        servo_set_config_mode(0);
        s_config_active = 0;
        dbg_print("[SERVO_TEST] config mode OFF\r\n");
    } else if (strcmp(token, "map") == 0) {
        if (strtok(NULL, " ") != NULL) goto usage;
        dbg_print("[SERVO_TEST] map shoulder=%u elbow=%u\r\n",
                  xb_servo[XB_SHOULDER].id, xb_servo[XB_ELBOW].id);
    } else {
        goto usage;
    }
    return;

usage:
    dbg_print("[SERVO_TEST] usage: servo ping|read|setid|bind|config|run|map | joint shoulder|elbow|gripper <pulse|+d|-d> | joint status | joint limit <j> <min> <max>\r\n");
    dbg_print("[SERVO_TEST] usage: yaw enable|disable|status|target <ecd>|step <d>|probe|dir <1|-1>|limit <n>|speed <rpm>|gain <kp> <kd> <kp> <kd>|fault [clear]|monitor|stop|set|clear\r\n");
    dbg_print("[SERVO_TEST] usage: stop | mode manual|protect|status | teach status|pt <n>|enter|exit|record|save|play|stop|load|erase|origin | flash test\r\n");
    dbg_print("[SERVO_TEST] usage: chassis status|line|stop|motor <0..3> <cur>|pid [line|0..3 kp ki kd]|map [vx|vy|wz <1|-1>] | can stat|recover <1|2>|raw <ch> <id> [hex..] | uart | rc\r\n");
    dbg_print("[SERVO_TEST] usage: pick status|learn <0..4>|run|time <n> <ms>|open <p>|close <p>|auto on|off|vis <x> <y>|apply|save | ik status|l1|l2|learn|xy|vis|depth|k|dy|yawk|save | vis | w25 id|test\r\n");
    dbg_print("[SERVO_TEST] usage: wp <0..4> <sh> <el>   set pick waypoint by pulse numbers, no servo read needed\r\n");
}

/* 行组装。USART10 中断和 USB 中断都会调，所以进临界区。 */
static void console_rx_byte(char ch)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();

    if (ch == '\r' || ch == '\n') {
        if (s_line_len > 0U) {
            servo_test_msg_t msg;
            memcpy(msg.line, s_line, s_line_len);
            msg.line[s_line_len] = '\0';
            if (s_queue != NULL) {
                (void)xQueueSendFromISR(s_queue, &msg, NULL);
            }
            s_line_len = 0U;
        }
    } else if (s_line_len + 1U < sizeof(s_line)) {
        s_line[s_line_len++] = ch;
    } else {
        s_line_len = 0U;
    }
    __set_PRIMASK(pm);
}

/* USB CDC 中断里逐字节调用 */
void servo_test_rx_byte(char ch)
{
    s_rx_bytes++;
    console_rx_byte(ch);
}

void servo_test_rx_isr(void)
{
    uint32_t isr = USART10->ISR;
    if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE)) {
        __HAL_UART_CLEAR_FLAG(&huart10, UART_CLEAR_OREF);
        __HAL_UART_CLEAR_FLAG(&huart10, UART_CLEAR_FEF);
        __HAL_UART_CLEAR_FLAG(&huart10, UART_CLEAR_NEF);
    }
    while (USART10->ISR & USART_ISR_RXNE_RXFNE) {
        s_rx_bytes++;
        console_rx_byte((char)USART10->RDR);
    }
}

void servo_test_init(void)
{
    s_queue = xQueueCreate(SERVO_TEST_QUEUE_LEN, sizeof(servo_test_msg_t));
    s_line_len = 0;
    s_config_allowed = 0;
    s_config_active = 0;
    __HAL_UART_DISABLE_IT(&huart10, UART_IT_RXNE);
    while (USART10->ISR & USART_ISR_RXNE_RXFNE) (void)USART10->RDR;
    __HAL_UART_CLEAR_FLAG(&huart10, UART_CLEAR_OREF);
    __HAL_UART_CLEAR_FLAG(&huart10, UART_CLEAR_FEF);
    __HAL_UART_CLEAR_FLAG(&huart10, UART_CLEAR_NEF);
    HAL_NVIC_SetPriority(USART10_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART10_IRQn);
    __HAL_UART_ENABLE_IT(&huart10, UART_IT_RXNE);
}

void servo_test_process(void)
{
    servo_test_msg_t msg;
    if (s_queue == NULL || xQueueReceive(s_queue, &msg, 0) != pdTRUE) return;
    servo_test_parse_line(msg.line);
}

void servo_test_poll(void)
{
    if (!s_yaw_monitor_active) return;
    if (++s_yaw_monitor_div < (ARM_YAW_LIMIT_LOG_PERIOD_MS / ARM_TASK_PERIOD)) return;
    s_yaw_monitor_div = 0U;
    arm_yaw_limit_clamp_target(arm_yaw_feedback.total_count);
    dbg_print("[YAW_LIMIT] ecd=%ld total=%ld round=%ld angle=%.2f rpm=%d raw_i=%d temp=%u rx=%lu online=%u limits=%d side=%d\r\n",
              (long)arm_yaw_feedback.ecd,
              (long)arm_yaw_feedback.total_count,
              (long)arm_yaw_feedback.round_count,
              arm_yaw_feedback.angle_deg,
              arm_yaw_feedback.speed_rpm,
              arm_yaw_feedback.current_raw,
              (unsigned)arm_yaw_feedback.temperature,
              (unsigned long)arm_yaw_feedback.receive_count,
              (unsigned)arm_yaw_feedback.online,
              arm_yaw_limit_config.configured,
              (int)arm_yaw_limit_side());
}

void servo_test_set_config_allowed(uint8_t allowed)
{
    s_config_allowed = allowed ? 1U : 0U;
}

uint8_t servo_test_config_active(void)
{
    return s_config_active;
}
