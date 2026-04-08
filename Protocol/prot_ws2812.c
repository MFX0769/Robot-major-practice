#include "prot_ws2812.h"
#include "spi.h"
#include "dma.h"

#define brightness 0x50 //亮度 0x00-0xff
const uint8_t code[] = {0xC0, 0xF8};
extern DMA_HandleTypeDef hdma_spi1_tx;

rgb_t rgb = {0};
Color_t color;
void rgb_set_color(uint8_t id, rgb_color_t color)
{
    if (id < RGB_NUM) {
        rgb.rgb_data[id].g = color.g;
        rgb.rgb_data[id].b = color.b;
        rgb.rgb_data[id].r = color.r;
    }
}

void rgb_reflash(uint8_t reflash_num)
{
    uint8_t data_b, data_r, data_g;
    for (int i = 0; i < reflash_num; i++) {
        data_g = rgb.rgb_data[i].g;
        data_r = rgb.rgb_data[i].r;
        data_b = rgb.rgb_data[i].b;
        for (int j = 0; j < 8; j++) {
            rgb.rgb_buffer[24 * i + 7 - j + ZERO_NUM] = code[data_g & 0x01];
            rgb.rgb_buffer[24 * i + 15 - j + ZERO_NUM] = code[data_r & 0x01];
            rgb.rgb_buffer[24 * i + 23 - j + ZERO_NUM] = code[data_b & 0x01];
            data_g >>= 1; data_r >>= 1; data_b >>= 1;
        }
    }
    HAL_SPI_Transmit_DMA(&hspi1, rgb.rgb_buffer, 24 * reflash_num  + 2 * ZERO_NUM);
}

void rgb_change(uint8_t i, uint8_t status)
{
    rgb.rgb_status[i] = status;
}

void rgb_set_bright(uint8_t i, uint8_t bright)
{
    rgb_set_color(i, (rgb_color_t){bright,bright,bright});
}

void update_leds(uint8_t led_num,uint8_t status)
{
	for (uint8_t i = 0; i < led_num; i++) {
		rgb_change(i,status);
    }
}

void led_range_on(uint8_t start_led, uint8_t end_led, Color_t color)
{
    // 先全部熄灭
    update_leds(RGB_NUM, OFF);
    
    // 点亮指定范围
    for (uint8_t i = start_led; i <= end_led && i < RGB_NUM; i++) {
        rgb_change(i, color);
    }
}

/**
 * @brief 清除所有灯
 */
void clear_all_leds(void)
{
    for (uint8_t i = 0; i < RGB_NUM; i++) {
        rgb_change(i, OFF);
    }
}

//void led_flow(uint8_t led_num, Color_t color, uint8_t direction, uint16_t speed_ms)
//{
//    static uint8_t current_led = 0;
//    static uint32_t last_time = 0;
//    
//    uint32_t now = HAL_GetTick();
//    
//    // 时间控制
//    if (now - last_time < speed_ms) return;
//    
//    last_time = now;
//    
//    // 移到下一个灯
//    current_led++;
//    if (current_led > led_num) {
//		update_leds(led_num,OFF);
//        current_led = 0;		
//    }
//    
//    // 显示
//    if (direction == 0) {  // 从左到右
//        update_leds(current_led, color);
//    } else {  // 从右到左
//		update_leds(led_num,OFF);
//        update_leds(led_num - current_led, color);
//    }
//    
//}

void led_flow(uint8_t start_led, uint8_t end_led, Color_t color, uint8_t direction, uint16_t speed_ms)
{
    static uint8_t current_led = 0;
    static uint32_t last_time = 0;
    
    uint32_t now = HAL_GetTick();
    uint8_t led_num = end_led - start_led + 1;
    
    if (now - last_time < speed_ms) return;
    last_time = now;
    
    // 清除整个范围
    for (uint8_t i = start_led; i <= end_led; i++) {
        rgb_change(i, OFF);
    }
    
    // 计算当前要点亮的灯
    uint8_t actual_led;
    if (direction == 0) {  // 左到右
        actual_led = start_led + current_led;
        current_led = (current_led + 1) % led_num;
    } else {  // 右到左
        actual_led = end_led - current_led;
        current_led = (current_led + 1) % led_num;
    }
    
    // 只点亮当前这一个灯
    rgb_change(actual_led, color);
}

void led_double_flash(uint8_t start_led, uint8_t end_led, Color_t color, uint16_t on_ms, uint16_t off_ms)
{
    static uint8_t state = 0;  // 0=灭, 1=亮
    static uint32_t last_time = 0;
    
    uint32_t now = HAL_GetTick();
    uint16_t delay = state ? on_ms : off_ms;
    
    if (now - last_time < delay) return;
    last_time = now;
    
    if (state) {
        // 亮：点亮范围
        for (uint8_t i = start_led; i <= end_led && i < RGB_NUM; i++) {
            rgb_change(i, color);
        }
    } else {
        // 灭：熄灭范围
        for (uint8_t i = start_led; i <= end_led && i < RGB_NUM; i++) {
            rgb_change(i, OFF);
        }
    }
    
    state = !state;
}

void rgb_output_data(void)
{
    for (int i = 0; i < RGB_NUM; i++) {
        switch (rgb.rgb_status[i]) {
            case OFF: rgb_set_color(i, (rgb_color_t){0,0,0});break;                           //无色
            case RED: rgb_set_color(i, (rgb_color_t){brightness,0,0});break;                  //红色
            case GREEN: rgb_set_color(i, (rgb_color_t){0,brightness,0});break;                  //绿色
            case BLUE: rgb_set_color(i, (rgb_color_t){0,0,brightness});break;                  //蓝色
            case YELLOW: rgb_set_color(i, (rgb_color_t){brightness,brightness,0});break;         //黄色
            case PURPLE: rgb_set_color(i, (rgb_color_t){brightness,0,brightness});break;         //紫色
            case CYAN: rgb_set_color(i, (rgb_color_t){0,brightness,brightness});break;         //靛色
            case WHITE: rgb_set_color(i, (rgb_color_t){brightness,brightness,brightness});break;//白色
            case 8: break;
        }
    }
    rgb_reflash(RGB_NUM);
}

