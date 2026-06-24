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
} prot_8irtraking_data_t;

void prot_8irtraking_init(void);
void prot_8irtraking_cmd_read(uint8_t analog_en, uint8_t digital_en);
void prot_8irtraking_on_receive(const uint8_t *buf, uint16_t len);
const prot_8irtraking_data_t *prot_8irtraking_get_data(void);

#endif