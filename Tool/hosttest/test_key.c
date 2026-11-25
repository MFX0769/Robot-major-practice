/* 主机测试：把真实的 Bsp/bsp_user_key.c 原样包含进来，按时间脚本喂按键电平。
   跑法：py Tool\\hosttest\\run.py */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sim.h"
#include "../../Bsp/bsp_user_key.c"    /* 真实固件源文件 */

static int g_fail, g_pass;
static const char *g_section = "";

#define CHECK(c, msg) do { if (c) { g_pass++; } else { g_fail++; printf("    FAIL [%s] %s (line %d)\n", g_section, msg, __LINE__); } } while (0)

/* 事件记录 */
typedef struct { uint32_t t; int ev; } ev_log_t;
static ev_log_t s_ev[64];
static uint32_t s_ev_n;
static uint32_t s_press_t0;

static const char *ev_name(int e)
{
    switch (e) {
        case KEY_EVT_NONE:   return "NONE";
        case KEY_EVT_SHORT:  return "SHORT";
        case KEY_EVT_DOUBLE: return "DOUBLE";
        case KEY_EVT_TRIPLE: return "TRIPLE";
        case KEY_EVT_LONG2:  return "LONG2";
        case KEY_EVT_LONG5:  return "LONG5";
        default:             return "?";
    }
}

/* 每 3ms 轮询一次（与机械臂任务一致），共 ms 毫秒 */
static void poll_for(uint32_t ms)
{
    uint32_t end = g_now_ms + ms;

    while (g_now_ms < end) {
        key_event_e e = BSP_UserKey_Poll();
        if (e != KEY_EVT_NONE && s_ev_n < 64U) {
            s_ev[s_ev_n].t = g_now_ms;
            s_ev[s_ev_n].ev = (int)e;
            s_ev_n++;
        }
        sim_advance(3U);
    }
}

static void down(void) { g_btn_pressed = 1; s_press_t0 = g_now_ms; }
static void up(void)   { g_btn_pressed = 0; }

static void section(const char *name)
{
    g_section = name;
    s_ev_n = 0U;
    printf("== %s\n", name);
}

static int count_ev(int e)
{
    uint32_t i;
    int n = 0;
    for (i = 0U; i < s_ev_n; i++) if (s_ev[i].ev == e) n++;
    return n;
}

static void show(void)
{
    uint32_t i;
    printf("    events:");
    for (i = 0U; i < s_ev_n; i++) printf(" %s@%lu", ev_name(s_ev[i].ev), (unsigned long)s_ev[i].t);
    printf("\n");
}

int main(void)
{
    int i;

    sim_reset();
    g_verbose = 0;
    BSP_UserKey_Init();
    poll_for(100U);
    CHECK(s_ev_n == 0U, "idle button produces no event");

    section("K1 单击");
    down(); poll_for(80U); up(); poll_for(600U);
    show();
    CHECK(count_ev(KEY_EVT_SHORT) == 1, "one SHORT");
    CHECK(count_ev(KEY_EVT_DOUBLE) == 0 && count_ev(KEY_EVT_LONG2) == 0, "nothing else");

    section("K2 双击");
    down(); poll_for(80U); up(); poll_for(120U); down(); poll_for(80U); up(); poll_for(600U);
    show();
    CHECK(count_ev(KEY_EVT_DOUBLE) == 1, "one DOUBLE");
    CHECK(count_ev(KEY_EVT_SHORT) == 0, "no stray SHORT");

    section("K3 三击");
    down(); poll_for(80U); up(); poll_for(120U); down(); poll_for(80U); up(); poll_for(120U); down(); poll_for(80U); up(); poll_for(600U);
    show();
    CHECK(count_ev(KEY_EVT_TRIPLE) == 1, "one TRIPLE");
    CHECK(count_ev(KEY_EVT_SHORT) == 0 && count_ev(KEY_EVT_DOUBLE) == 0, "nothing else");

    section("K4 长按 2.3 秒");
    down(); poll_for(2300U); up(); poll_for(600U);
    show();
    CHECK(count_ev(KEY_EVT_LONG2) == 1, "one LONG2");
    CHECK(count_ev(KEY_EVT_LONG5) == 0 && count_ev(KEY_EVT_SHORT) == 0, "nothing else");
    CHECK(s_ev[0].t - s_press_t0 >= 2300U, "LONG2 is reported on release, not while held");

    section("K5 按住 1.9 秒不算长按");
    down(); poll_for(1900U); up(); poll_for(600U);
    show();
    CHECK(count_ev(KEY_EVT_LONG2) == 0, "no LONG2");
    CHECK(count_ev(KEY_EVT_SHORT) == 1, "counted as a click");

    section("K6 长按 5.3 秒");
    down(); poll_for(5300U); up(); poll_for(600U);
    show();
    CHECK(count_ev(KEY_EVT_LONG5) == 1, "one LONG5");
    CHECK(count_ev(KEY_EVT_LONG2) == 0 && count_ev(KEY_EVT_SHORT) == 0, "no LONG2 / click on release");
    CHECK(s_ev[0].t - s_press_t0 < 5100U, "LONG5 reported while still held, about 5.0s");

    section("K7 抖动：几个 3ms 的毛刺不产生事件");
    for (i = 0; i < 8; i++) { down(); poll_for(3U); up(); poll_for(3U); }
    poll_for(600U);
    show();
    CHECK(s_ev_n == 0U, "no event from contact bounce");

    section("K8 删除确认：5 秒内再来一次长按 5 秒是可行的");
    down(); poll_for(5300U); up(); poll_for(400U);
    down(); poll_for(5300U); up(); poll_for(600U);
    show();
    CHECK(count_ev(KEY_EVT_LONG5) == 2, "two LONG5 events");
    if (count_ev(KEY_EVT_LONG5) == 2) {
        printf("    gap between the two LONG5 = %lums (confirm window is 12000ms)\n",
               (unsigned long)(s_ev[1].t - s_ev[0].t));
        CHECK(s_ev[1].t - s_ev[0].t < 12000U, "second LONG5 arrives inside the confirm window");
    }

    section("K9 按住不放：4 秒处还没事件，到 5 秒只报一次");
    down(); poll_for(4000U);
    CHECK(s_ev_n == 0U, "nothing at 4.0s while still held");
    poll_for(6000U);
    CHECK(count_ev(KEY_EVT_LONG5) == 1, "exactly one LONG5 after holding 10s");
    up(); poll_for(600U);
    CHECK(count_ev(KEY_EVT_LONG5) == 1, "release does not add anything");

    printf("\n%d checks, %d failed\n", g_pass + g_fail, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
