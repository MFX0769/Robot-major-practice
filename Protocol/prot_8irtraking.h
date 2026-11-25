#ifndef __PROT_8IRTRAKING_H
#define __PROT_8IRTRAKING_H

#include <stdint.h>
#include <string.h>

#define PROT_8IR_NUM_SENSORS 8

/* 线状态: 0=丢线(position保持记忆值) 1=正常 2=全压线(十字/全黑,position保持) */
#define LINE_STATE_LOST   0
#define LINE_STATE_OK     1
#define LINE_STATE_ALLON  2

typedef struct {
    uint8_t digital[8];
    uint16_t analog[8];
    uint8_t state;
    int16_t position;   /* 模拟量加权质心 ±100, 丢线/全压线时保持记忆值 */
    uint32_t rx_cnt;
    uint8_t on_cnt;     /* 自适应阈值下有效通道数 */
    uint16_t contrast;  /* 本帧模拟量 max-min, 低于门限视为无线 */
} prot_8irtraking_data_t;

extern prot_8irtraking_data_t s_data;

void prot_8irtraking_init(void);
int  prot_8irtraking_cmd_read(uint8_t analog_en, uint8_t digital_en);
void prot_8irtraking_on_receive(const uint8_t *buf, uint16_t len);
const prot_8irtraking_data_t *prot_8irtraking_get_data(void);

void prot_8ir_start_rx(void);
void prot_8ir_on_rx(void);
void prot_8ir_uart_diag(void);
void prot_8ir_poll_rx(void);
extern volatile uint32_t ir_poll_bytes;
extern volatile uint32_t ir_isr_bytes;
extern volatile uint32_t ir_diag_ticks;
extern volatile uint32_t ir_last_isr;
extern volatile uint32_t ir_ore_count;
extern volatile uint32_t ir_pe7_changes;

#endif
