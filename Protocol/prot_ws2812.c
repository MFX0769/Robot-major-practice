#include "prot_ws2812.h"
#include "mode_switch_task.h"
#include "shoot_task.h"

uint8_t color_buff[LED_NUM*COLOR_LENGTH + 2]={0};//预计6个灯，一个灯24位数据
uint8_t reset_buff[100]={0};
Indicator_color_e color;



void Indicator_Init(void)
{
    memset(color_buff,0,sizeof(color_buff));         
    HAL_SPI_Transmit_DMA(&hspi1, color_buff, sizeof(color_buff));    
}

// 获取颜色RGB值
void get_color_rgb(Indicator_color_e color, uint8_t *r, uint8_t *g, uint8_t *b) {
    switch(color) {
        case RED:     *r=255; *g=0;   *b=0;   break;
        case GREEN:   *r=0;   *g=255; *b=0;   break;
        case BLUE:    *r=0;   *g=0;   *b=255; break;
        case YELLOW:  *r=255; *g=255; *b=0;   break;
        case ORANGE:  *r=255; *g=165; *b=0;   break;
        case CYAN:    *r=0;   *g=255; *b=255; break;
        case WHITE:   *r=255; *g=255; *b=255; break;
        case PURPLE:  *r=128; *g=0;   *b=128; break;
		case PINK:  *r=220; *g=100;   *b=20; break;
        case OFF:
        default:            *r=0;   *g=0;   *b=0;   break;
    }
}

void rgb_to_spi(uint8_t r, uint8_t g, uint8_t b, uint8_t *spi_data) {
    uint32_t grb = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
    
    for (int8_t bit = 23; bit >= 0; bit--) {
        spi_data[23 - bit] = (grb & (1UL << bit)) ? WS_BIT_1 : WS_BIT_0;
    }
}

// 更新num个灯珠到SPI缓冲区
static void update_led(uint8_t led_num, Indicator_color_e color) {
    if (led_num < 1 || led_num > LED_NUM) return;
    
    uint8_t r, g, b;
    get_color_rgb(color, &r, &g, &b);
    
    uint8_t *buf = &color_buff[(led_num - 1) * COLOR_LENGTH];
    rgb_to_spi(r, g, b, buf);
}

// 更新所有灯珠到SPI缓冲区
static void update_all_leds(Indicator_color_e color) {
    uint8_t r, g, b;
    get_color_rgb(color, &r, &g, &b);
    
    for (uint8_t i = 0; i < LED_NUM; i++) {
        uint8_t *buf = &color_buff[i * COLOR_LENGTH];
        rgb_to_spi(r, g, b, buf);
    }
}

// 发送数据到灯带
static void send_data(void) {
//    while(dma_busy) {
//        __NOP();
//    }
//    
//    dma_busy = true;
	
    HAL_SPI_Transmit_DMA(&hspi1, color_buff, sizeof(color_buff));
	
}

// ================= 你想要的API =================

// 1. 基础点亮：设置灯珠颜色并立即显示
void led_show(uint8_t led_num, Indicator_color_e color) {
	 update_all_leds(OFF);//先把所有都写逻辑0
    update_led(led_num, color);//再写入想要亮灯的逻辑
    send_data();
}

static uint32_t led_flow_cnt=0;

// 2. 流水灯：多次调用逐渐完成整个流水效果
void led_flow(uint8_t led_num,Indicator_color_e color, uint8_t direction, uint16_t speed_ms) {
	static uint8_t start_led=1;
    // 参数检查
    if (start_led > led_num) start_led = 1;
	led_flow_cnt++;
	if(led_flow_cnt % 100 ==0)
	{
		start_led++;
	}
    
    // 先关闭所有灯
    update_all_leds(OFF);
    send_data();
//    delay_ms(100);
    
    if (direction == 0) {  // 从左到右
		update_led(start_led,color);
		send_data();
//        for (uint8_t i = start_led; i <= LED_NUM; i++) {
//            update_led(i, color);
//            send_data();
////            delay_ms(speed_ms);
//            
//            // 可以添加关闭前一个灯的效果
//            if (i > start_led) {
//                update_led(i-1,OFF);
//            }
//        }
    } else {  // 从右到左
		update_all_leds(color);
		uint8_t end_led=LED_NUM-start_led;
		update_led(end_led,OFF);
		send_data();
		
//        for (int8_t i = (start_led == 0 ? LED_NUM : start_led); i >= 1; i--) {
//            update_led(i, color);
//            send_data();
////            delay_ms(speed_ms);
//            
//            // 可以添加关闭后一个灯的效果
//            if (i < (start_led == 0 ? LED_NUM : start_led)) {
//                update_led(i+1,OFF);
//            }
//        }
    }
}

// 3. 闪烁：调用一次就完成指定次数的闪烁
void led_flash(uint8_t led_num, Indicator_color_e color, uint8_t times, uint16_t interval_ms) {
    for (uint8_t i = 0; i < times; i++) {
        // 亮
        update_led(led_num, color);
        send_data();
//        delay_ms(interval_ms);
        
        // 灭（最后一次不灭）
        if (i < times - 1) {
            update_led(led_num, OFF);
            send_data();
//            delay_ms(interval_ms);
        }
    }
}

