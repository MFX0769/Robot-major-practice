/**
  * @file prot_dbus.h
  * @brief DBUS 遥控器接收接口 (UART5/PD2, 100000-8E1)
 */
#ifndef PROT_DBUS_H
#define PROT_DBUS_H

#include <stdint.h>

#define DBUS_FRAME_LEN 18U
#define DBUS_QUEUE_LEN  8U

typedef struct {
    uint8_t data[DBUS_FRAME_LEN];
} dbus_frame_t;

void dbus_init(void);
void dbus_rx_isr(void);
void dbus_process(void);
uint8_t dbus_online(void);
uint32_t dbus_rx_count(void);
uint32_t dbus_rx_bytes(void);

#endif
