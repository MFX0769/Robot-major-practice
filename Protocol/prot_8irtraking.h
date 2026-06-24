#ifndef __PROT_8IRTRAKING_H
#define __PROT_8IRTRAKING_H

#include <stdint.h>
#include <string.h>

#define PROT_8IR_NUM_SENSORS 8

typedef struct {
    uint8_t digital[8];
    uint16_t analog[8];
    uint8_t state;
    int16_t position;
    uint32_t rx_cnt;  /* on_receive()被调用次数，用来确认USART10的IDLE回调是否真的进过 */
} prot_8irtraking_data_t;

extern prot_8irtraking_data_t s_data;

void prot_8irtraking_init(void);
int prot_8irtraking_cmd_read(uint8_t analog_en, uint8_t digital_en);
void prot_8irtraking_on_receive(const uint8_t *buf, uint16_t len);
void prot_8irtraking_poll_gpio(void);
const prot_8irtraking_data_t *prot_8irtraking_get_data(void);

#endif