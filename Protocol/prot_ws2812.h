#ifndef __PROT_WS2812_H
#define __PROT_WS2812_H

#include "stdint.h"

#define RGB_NUM 32
#define ZERO_NUM 30
typedef struct
{
    uint8_t r, g, b;
} rgb_color_t;

typedef struct
{
    uint8_t rgb_status[RGB_NUM];
    uint8_t rgb_buffer[24 * RGB_NUM + 2 * ZERO_NUM];
    rgb_color_t rgb_data[RGB_NUM];
} rgb_t;

typedef enum {
    OFF = 0,      // 无色/关闭
    RED,          // 红色
    GREEN,        // 绿色
    BLUE,         // 蓝色
    YELLOW,       // 黄色
    PURPLE,       // 紫色
    CYAN,         // 靛色/青色
    WHITE,        // 白色
    MAX           // 颜色总数，用于边界检查
}Color_t;

extern rgb_t rgb;

void rgb_change(uint8_t i, uint8_t status);
void rgb_set_bright(uint8_t i, uint8_t bright);
void update_leds(uint8_t led_num,uint8_t status);
void led_range_on(uint8_t start_led, uint8_t end_led, Color_t color);
void led_flow(uint8_t start_led, uint8_t end_led, Color_t color, uint8_t direction, uint16_t speed_ms);
void led_double_flash(uint8_t start_led, uint8_t end_led, Color_t color, uint16_t on_ms, uint16_t off_ms);
void clear_all_leds(void);

void rgb_output_data(void);

#endif
