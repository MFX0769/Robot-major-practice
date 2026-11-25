#include "prot_sbus.h"
#include "prot_dr16.h"
#include "prot_dbus.h"
/* DBUS compatibility layer: existing chassis/arm logic keeps wbus_rc API. */

wbus_frame_u wbus_rc;
volatile uint32_t wbus_rx_cnt;
volatile float wbus_rx_freq;

static uint8_t dbus_sw_to_wbus(uint8_t sw)
{
    /* DBUS: 1上/2下/3中; 原 wbus: 2上/0下/1中。 */
    if (sw == 1U) return 2U;
    if (sw == 2U) return 0U;
    return 1U;
}

void sbus_callback_handler(uint8_t *buff, uint16_t len)
{
    if (buff == NULL || len < DBUS_FRAME_LEN) return;
    rc_callback_handler(&rc, buff);
    wbus_rc.data.ch1 = rc.ch1;
    wbus_rc.data.ch2 = rc.ch2;
    wbus_rc.data.ch3 = rc.ch3;
    wbus_rc.data.ch4 = rc.ch4;
    wbus_rc.data.ch5 = rc.ch5;
    wbus_rc.data.sw1 = dbus_sw_to_wbus(rc.sw1);
    wbus_rc.data.sw2 = dbus_sw_to_wbus(rc.sw2);
    wbus_rc.data.sw3 = 1;  /* DBUS 无独立 SWD, 由左拨杆解锁状态控制机械臂供电 */
    wbus_rx_cnt++;
}
