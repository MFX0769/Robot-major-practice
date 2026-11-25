/**
  * @file tfmini.h
  * @brief 北醒 TF-mini Plus 四路测距接口
  * @note UART 115200-8N1, 9B 帧: 59 59 distL distH strL strH tempL tempH sum。
  */
#ifndef TFMINI_H
#define TFMINI_H

#include <stdint.h>

#define TFMINI_NUM       4U
#define TFMINI_FRAME_LEN 9U
#define TFMINI_MAX_AGE_MS 150U

typedef enum {
    TFMINI_FRONT = 0,
    TFMINI_LEFT,
    TFMINI_RIGHT,
    TFMINI_REAR,
} tfmini_id_e;

typedef struct {
    uint16_t distance_mm;
    uint16_t strength;
    int16_t temp_c;
    uint32_t last_tick;
    uint32_t rx_frames;
    uint32_t crc_errors;
} tfmini_data_t;

extern tfmini_data_t tfmini[TFMINI_NUM];

void tfmini_init(void);
void tfmini_poll(void);
uint8_t tfmini_online(uint8_t id);
uint16_t tfmini_distance(uint8_t id);
void tfmini_rx_isr(uint8_t id);

#endif
