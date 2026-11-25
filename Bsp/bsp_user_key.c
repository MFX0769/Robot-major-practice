/**
  * @file    bsp_user_key.c
  * @brief   PA15 单按钮事件扫描
  * @note    15ms 去抖；短按动作延迟 300ms 以区分双击/三击；
  *          长按 5s 在按住时立即上报，长按 2s 在松开时上报。
  */
#include "bsp_user_key.h"

#define KEY_DEBOUNCE_MS   15U
#define KEY_CLICK_WINDOW  300U
#define KEY_LONG2_MS      2000U
#define KEY_LONG5_MS      5000U

static uint8_t  s_stable;          /* 1=按下 */
static uint8_t  s_raw_last;
static uint32_t s_raw_change_tick;
static uint32_t s_press_tick;
static uint32_t s_last_release_tick;
static uint8_t  s_click_count;
static uint8_t  s_long5_fired;
static uint8_t  s_inited;

static uint8_t read_pressed(void)
{
    return (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET) ? 1U : 0U;
}

void BSP_UserKey_Init(void)
{
    GPIO_InitTypeDef g = {0};

    /* PA15 是 JTDI，复位默认的内部上拉被 CubeMX 的 NOPULL 关掉了。这里补回，
       避免外部上拉缺失时引脚悬空、产生假按键。低电平 = 按下。 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    g.Pin = GPIO_PIN_15;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &g);

    s_stable = 0U;
    s_raw_last = read_pressed();
    s_raw_change_tick = HAL_GetTick();
    s_press_tick = 0U;
    s_last_release_tick = 0U;
    s_click_count = 0U;
    s_long5_fired = 0U;
    s_inited = 1U;
}

key_event_e BSP_UserKey_Poll(void)
{
    uint32_t now;
    uint8_t raw;

    if (!s_inited) BSP_UserKey_Init();
    now = HAL_GetTick();
    raw = read_pressed();

    /* 去抖 */
    if (raw != s_raw_last) {
        s_raw_last = raw;
        s_raw_change_tick = now;
    } else if (raw != s_stable && (uint32_t)(now - s_raw_change_tick) >= KEY_DEBOUNCE_MS) {
        s_stable = raw;
        if (s_stable) {
            s_press_tick = now;
            s_long5_fired = 0U;
        } else {
            uint32_t hold = (uint32_t)(now - s_press_tick);
            if (hold >= KEY_LONG2_MS && hold < KEY_LONG5_MS) {
                s_click_count = 0U;             /* 长按吃掉前面的点击 */
                return KEY_EVT_LONG2;
            }
            if (hold < KEY_LONG2_MS) {
                if (s_click_count < 3U) s_click_count++;
                s_last_release_tick = now;
            }
        }
    }

    /* 按住满 5s：立即上报 */
    if (s_stable && !s_long5_fired &&
        (uint32_t)(now - s_press_tick) >= KEY_LONG5_MS) {
        s_long5_fired = 1U;
        s_click_count = 0U;
        return KEY_EVT_LONG5;
    }

    /* 点击判定窗到期 */
    if (!s_stable && s_click_count > 0U &&
        (uint32_t)(now - s_last_release_tick) >= KEY_CLICK_WINDOW) {
        uint8_t n = s_click_count;
        s_click_count = 0U;
        if (n >= 3U) return KEY_EVT_TRIPLE;
        if (n == 2U) return KEY_EVT_DOUBLE;
        return KEY_EVT_SHORT;
    }

    return KEY_EVT_NONE;
}
