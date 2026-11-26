/* 主机场景测试：真实的 Protocol/prot_dbus.c（按空闲切帧）和 prot_sbus.c（开关映射）。
   模拟的是串口接收寄存器和中断入口；解码函数用替身，只检查它拿到的字节和开关映射。
   跑法：py Tool\\hosttest\\run.py --dbus */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sim.h"
#include "main.h"
#include "prot_dbus.h"
#include "prot_sbus.h"
#include "status_task.h"

static int g_fail, g_pass;
static const char *g_section = "";

#define CHECK(c, msg) do { if (c) { g_pass++; } else { g_fail++; printf("    FAIL [%s] %s (line %d)\n", g_section, msg, __LINE__); } } while (0)

static void section(const char *name)
{
    g_section = name;
    printf("== %s\n", name);
}

/* 模拟中断控制器：每个字节落到 RDR 就进一次中断；一帧发完总线空闲，再进一次 */
static void uart_deliver(const uint8_t *buf, uint32_t n)
{
    uint32_t i;

    for (i = 0U; i < n; i++) {
        sim_uart_rx_push(5, buf[i]);
        dbus_rx_isr();
    }
    sim_uart_rx_idle(5);
    dbus_rx_isr();
}

/* 中断被挡住一会儿：所有字节都到了才进一次中断 */
static void uart_deliver_late(const uint8_t *buf, uint32_t n)
{
    uint32_t i;

    for (i = 0U; i < n; i++) sim_uart_rx_push(5, buf[i]);
    dbus_rx_isr();
    sim_uart_rx_idle(5);
    dbus_rx_isr();
}

/* 模式任务每 5ms 取一次；这里一次把队列取空，返回解了几帧 */
static uint32_t drain(void)
{
    uint32_t before = g_sbus_calls;
    uint32_t i;

    for (i = 0U; i < 9U; i++) dbus_process();
    return g_sbus_calls - before;
}

static uint8_t s_frame[18];

/* 按大疆遥控器的位拼法组一帧 18 字节：通道是 11 位原始值，开关 1 上 / 2 下 / 3 中 */
static uint8_t *mk_frame(int ch1, int ch2, int ch3, int ch4, int sw1, int sw2)
{
    memset(s_frame, 0, sizeof(s_frame));
    s_frame[0] = (uint8_t)(ch1 & 0xFF);
    s_frame[1] = (uint8_t)((ch1 >> 8) | ((ch2 & 0x1F) << 3));
    s_frame[2] = (uint8_t)((ch2 >> 5) | ((ch3 & 0x03) << 6));
    s_frame[3] = (uint8_t)((ch3 >> 2) & 0xFF);
    s_frame[4] = (uint8_t)(((ch3 >> 10) & 0x01) | ((ch4 & 0x7F) << 1));
    s_frame[5] = (uint8_t)((ch4 >> 7) | (sw1 << 5) | (sw2 << 4));
    return s_frame;
}

int main(void)
{
    uint8_t *f;
    uint8_t big[64];
    uint32_t n0, i, expect18;
    uint32_t frames, bad, drop, last_ms, max_ms;

    sim_reset();
    g_verbose = (getenv("SIM_VERBOSE") != NULL);
    huart5.uart = 5;
    dbus_init();

    /* ------------------------------------------------------------------ */
    section("D1 上电");
    CHECK(dbus_rx_count() == 0 && dbus_rx_bytes() == 0, "no frame, no bytes");
    CHECK(dbus_online() == 0, "offline at boot");
    CHECK(drain() == 0, "dbus_process with an empty queue decodes nothing");

    /* ------------------------------------------------------------------ */
    section("D2 一帧 18 字节 + 空闲：收下，解码函数拿到的就是这 18 字节");
    sim_rc_set_sw2(3);
    f = mk_frame(1024, 1024, 1024, 1024, 3, 3);
    uart_deliver(f, 18U);
    CHECK(dbus_rx_count() == 1, "one frame counted");
    CHECK(dbus_rx_bytes() == 18, "18 bytes counted");
    CHECK(dbus_online() == 1, "online right after a frame");
    CHECK(g_sbus_calls == 0, "nothing decoded until the task calls dbus_process");
    CHECK(drain() == 1 && memcmp(g_rc_last, f, 18U) == 0, "decoder got exactly the 18 bytes that were sent");
    CHECK(wbus_rc.data.sw2 == 1 && wbus_rc.data.sw1 == 1, "switch middle (3) -> wbus 1");
    CHECK(wbus_rx_cnt == 1, "frame counted by the sbus layer");
    CHECK((g_wdg_bits & WDG_BIT_BSP_REMOTE) != 0U, "watchdog bit fed");
    CHECK(drain() == 0, "queue emptied: the next process call decodes nothing");

    sim_rc_set_sw2(1);
    f = mk_frame(1024, 1024, 1024, 1024, 1, 1);
    uart_deliver(f, 18U);
    drain();
    CHECK(wbus_rc.data.sw2 == 2, "switch up (1) -> wbus 2 = chassis REMOTER mode");
    sim_rc_set_sw2(2);
    f = mk_frame(1024, 1024, 1024, 1024, 2, 2);
    uart_deliver(f, 18U);
    drain();
    CHECK(wbus_rc.data.sw2 == 0, "switch down (2) -> wbus 0 = chassis LINE_TRACK mode");

    /* ------------------------------------------------------------------ */
    section("D3 断联判定：100ms 没有新帧算掉线");
    sim_advance(90U);
    CHECK(dbus_online() == 1, "still online after 90ms");
    sim_advance(20U);
    CHECK(dbus_online() == 0, "offline after 100ms without frames");

    f = mk_frame(1024, 1024, 1024, 1024, 3, 3);

    /* ------------------------------------------------------------------ */
    section("D4 短帧丢掉，下一帧照常收");
    n0 = dbus_rx_count();
    uart_deliver(f, 17U);
    CHECK(dbus_rx_count() == n0, "17-byte burst dropped");
    uart_deliver(f, 18U);
    CHECK(dbus_rx_count() == n0 + 1U, "next good frame accepted");
    CHECK(drain() == 1 && memcmp(g_rc_last, f, 18U) == 0, "and delivered intact");

    /* ------------------------------------------------------------------ */
    section("D5 长帧（多一个字节）丢掉，之后能重新对齐");
    n0 = dbus_rx_count();
    memcpy(big, f, 18U);
    big[18] = 0x5AU;
    uart_deliver(big, 19U);
    CHECK(dbus_rx_count() == n0, "19-byte burst dropped");
    uart_deliver(f, 18U);
    CHECK(dbus_rx_count() == n0 + 1U, "resynced on the next 18-byte frame");
    CHECK(drain() == 1, "one frame decoded");

    /* ------------------------------------------------------------------ */
    section("D6 切帧不看内容：摇杆一动首字节就变，照样收");
    n0 = dbus_rx_count();
    uart_deliver(mk_frame(0, 1024, 1024, 1024, 3, 3), 18U);       /* 首字节 0x00 */
    uart_deliver(mk_frame(127, 1024, 1024, 1024, 3, 3), 18U);     /* 首字节 0x7F */
    uart_deliver(mk_frame(255, 1684, 364, 1500, 3, 3), 18U);      /* 首字节 0xFF */
    uart_deliver(mk_frame(12, 1024, 1024, 1024, 3, 3), 18U);      /* 首字节 0x0C，像帧头 */
    CHECK(dbus_rx_count() == n0 + 4U, "all four accepted");
    CHECK(drain() == 4, "all four decoded");

    /* ------------------------------------------------------------------ */
    section("D7 帧中途出错（溢出）：这一帧作废，下一帧正常");
    n0 = dbus_rx_count();
    for (i = 0U; i < 5U; i++) {
        sim_uart_rx_push(5, f[i]);
        dbus_rx_isr();
    }
    sim_uart_rx_error(5, USART_ISR_ORE);
    dbus_rx_isr();
    for (; i < 18U; i++) {
        sim_uart_rx_push(5, f[i]);
        dbus_rx_isr();
    }
    sim_uart_rx_idle(5);
    dbus_rx_isr();
    CHECK(dbus_rx_count() == n0, "frame with an overrun inside dropped");
    uart_deliver(f, 18U);
    CHECK(dbus_rx_count() == n0 + 1U, "next frame accepted after clearing the error flags");
    CHECK(drain() == 1 && memcmp(g_rc_last, f, 18U) == 0, "and intact");

    /* ------------------------------------------------------------------ */
    section("D8 帧头就带格式错：整帧作废");
    n0 = dbus_rx_count();
    for (i = 0U; i < 18U; i++) sim_uart_rx_push(5, f[i]);
    sim_uart_rx_error(5, USART_ISR_FE | USART_ISR_NE);
    dbus_rx_isr();
    sim_uart_rx_idle(5);
    dbus_rx_isr();
    CHECK(dbus_rx_count() == n0, "framing/noise error poisons the burst");
    uart_deliver(f, 18U);
    CHECK(dbus_rx_count() == n0 + 1U, "next frame accepted");
    CHECK(drain() == 1, "one frame decoded");

    /* ------------------------------------------------------------------ */
    section("D9 空闲中断多来一次没有影响");
    n0 = dbus_rx_count();
    for (i = 0U; i < 5U; i++) {
        sim_uart_rx_idle(5);
        dbus_rx_isr();
    }
    CHECK(dbus_rx_count() == n0, "spurious idle with an empty buffer is harmless");
    CHECK(drain() == 0, "nothing decoded");

    /* ------------------------------------------------------------------ */
    section("D10 中断被挡住：18 个字节都到了才进一次中断，照样收");
    n0 = dbus_rx_count();
    g_rc_last[0] ^= 0xFFU;                                     /* 先弄脏，免得拿上一帧的结果蒙混过关 */
    uart_deliver_late(f, 18U);
    CHECK(dbus_rx_count() == n0 + 1U, "accepted");
    CHECK(drain() == 1 && memcmp(g_rc_last, f, 18U) == 0, "decoded, bytes in order");

    /* ------------------------------------------------------------------ */
    section("D11 两帧连着发、中间没有空闲：分不开，丢掉；下一帧恢复");
    n0 = dbus_rx_count();
    memcpy(big, f, 18U);
    memcpy(big + 18, f, 18U);
    uart_deliver(big, 36U);
    CHECK(dbus_rx_count() == n0, "36 bytes in one burst dropped (content cannot split it)");
    uart_deliver(f, 18U);
    CHECK(dbus_rx_count() == n0 + 1U, "next frame accepted");
    CHECK(drain() == 1, "one frame decoded");

    /* ------------------------------------------------------------------ */
    section("D12 乱数据灌 2 秒：不崩、不越界，只有正好 18 字节的那一段才算一帧");
    n0 = dbus_rx_count();
    expect18 = 0U;
    srand(7);
    for (i = 0U; i < 1000U; i++) {
        uint32_t len = 1U + (uint32_t)(rand() % 40);
        uint32_t k;
        if (len == 18U) expect18++;
        for (k = 0U; k < len; k++) big[k] = (uint8_t)rand();
        uart_deliver(big, len);
        (void)drain();                                         /* 任务一直在取 */
    }
    CHECK(dbus_rx_count() == n0 + expect18, "accepted exactly the 18-byte bursts");
    CHECK(g_sbus_calls >= (int)(n0 + expect18), "every accepted frame was decoded");
    CHECK(dbus_rx_bytes() > 20000U, "every byte was counted");

    /* ------------------------------------------------------------------ */
    section("D13 队列 8 帧深：没人取的话第 9 帧丢掉，不写坏内存");
    n0 = dbus_rx_count();
    for (i = 0U; i < 9U; i++) uart_deliver(f, 18U);
    CHECK(dbus_rx_count() == n0 + 9U, "9 frames arrived");
    CHECK(drain() == 8, "8 decoded, the 9th was dropped (the mode task drains every 5ms, a frame comes every 14ms)");
    CHECK(drain() == 0, "queue empty");
    dbus_stats_get(&frames, &bad, &drop, &last_ms, &max_ms);
    CHECK(drop >= 1U && last_ms == 0U, "queue overflow is counted; back-to-back frames have 0ms spacing");
    sim_advance(14U);
    uart_deliver(f, 18U);
    drain();
    dbus_stats_get(&frames, &bad, &drop, &last_ms, &max_ms);
    CHECK(last_ms >= 14U && last_ms <= 17U, "after a spaced frame, interval timing becomes meaningful");

    /* ------------------------------------------------------------------ */
    section("D14 DBUS 统计：坏帧长和真实帧间隔（接收机约 14ms/帧）");
    bad = 0U;
    for (i = 0U; i < 10U; i++) uart_deliver(f, 17U);
    dbus_stats_get(&frames, &bad, &drop, &last_ms, &max_ms);
    CHECK(bad >= 10U, "ten 17-byte bursts counted as bad frames");
    for (i = 0U; i < 4U; i++) {
        sim_advance(14U);
        uart_deliver(f, 18U);
        drain();
    }
    dbus_stats_get(&frames, &bad, &drop, &last_ms, &max_ms);
    CHECK(last_ms >= 14U && last_ms <= 17U && max_ms >= last_ms,
          "valid frames record their real spacing (14ms simulated -> last/max interval)");

    printf("\n%d checks, %d failed\n", g_pass + g_fail, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
