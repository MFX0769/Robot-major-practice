#ifndef __PROT_WS2812__
#define __PROT_WS2812__

#include "spi.h"
#include "string.h"

#define WS_BIT_1   0xF8    // 1110 0000 (高电平873ns)
#define WS_BIT_0   0xC0    // 1000 0000 (高电平291ns)  //此处为spi频率为6

#define LED_NUM 6
#define COLOR_LENGTH 24

typedef enum 
{
	WHITE,
	RED,
	GREEN,
	BLUE,
	PURPLE,
	PINK,
	ORANGE,
	YELLOW,
	CYAN,
OFF,
} Indicator_color_e;

void Indicator_Init(void);

void led_show(uint8_t led_num, Indicator_color_e color);
void led_flow(uint8_t led_num, Indicator_color_e color, uint8_t direction, uint16_t speed_ms);
void led_flash(uint8_t led_num, Indicator_color_e color, uint8_t times, uint16_t interval_ms);


void turn_on_single_light(uint8_t num,Indicator_color_e color);
void turn_on_all_light(uint16_t RED,uint16_t GREEN,uint16_t BLUE);
void turn_on_single_RGB(uint8_t led_num,uint16_t R,uint16_t G,uint16_t B);




#endif

