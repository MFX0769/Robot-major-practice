#ifndef FAKE_PROT_VISION_H
#define FAKE_PROT_VISION_H
#include <stdint.h>
typedef struct {
    uint8_t flag;
    int16_t x_dis;
    int16_t y_dis;
    uint32_t rec_cnt;
    uint8_t target_type;
} vision_fruit_msg_t;
extern vision_fruit_msg_t vision_fruit;
extern volatile uint32_t vision_uart_rx_bytes;
extern volatile uint32_t vision_uart_rx_frames;
extern volatile uint32_t vision_uart_last_tick;
void vision_uart_init(void);
#endif
