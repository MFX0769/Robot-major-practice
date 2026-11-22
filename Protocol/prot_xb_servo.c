/**
  * @file    prot_xb_servo.c
  * @brief   ĞÇßÂ SP ÏµÁĞµ¥Ïß×ÜÏß¶æ»úÇı¶¯
  * @note    ÎïÀí²ã£ºUART4 @ PC10£¨AF8£©£¬¿ª HDSEL µ¥Ïß°ëË«¹¤¡ª¡ªTX ½Å¼´×ÜÏß£¬
  *          ¶æ»úĞÅºÅÏß½Ó PC10£¬¶à¶æ»ú¿¿¶æ»ú×ÔÉíË« 5264 ½Ó¿Ú¼¶Áª¡£
  *          Ïß³ÌÄ£ĞÍ£ºarm_task ÖÜÆÚµ÷ÓÃ poll()£»·¢ËÍÎª¶ÌÖ¡×èÈûÊ½£¨~15 ×Ö½Ú
  *          @115200 ¡Ö 1.3ms£©£¬·¢ËÍÆÚ¼ä¹Ø RE ÆÁ±Îµ¥Ïß»ØÉù£¬Ó¦´ğÓÉ RX ÖĞ¶Ï
  *          Èë»·ĞÎ»º³å£¬poll ½âÎö¡£
  *          ´ı¼¤»î×´Ì¬£ºÆôÓÃÌõ¼ş¼ûÍ·ÎÄ¼ş×¢ÊÍ¡£
  *          UART4 ÖĞ¶ÏÈë¿ÚÔÚ±¾ÎÄ¼ş¶¨Òå£¨ÏòÁ¿±í°´·ûºÅÁ´½Ó£¬²»¶¯ CubeMX ÎÄ¼ş£©¡£
  */
#include "prot_xb_servo.h"
#include "stm32h7xx_hal.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>

/* ---- Ä£¿éË½ÓĞ ---- */
static UART_HandleTypeDef xb_huart;
static uint8_t            xb_inited = 0;          /* init ³É¹¦ºó²ÅÔÊĞíÊÕ·¢ */
static volatile uint8_t   rx_buf[64];
static volatile uint8_t   rx_head = 0, rx_tail = 0;
static uint8_t            rx_byte;

xb_servo_dev_t xb_servo[XB_JOINT_NUM] = {
    {0, 0, XB_PULSE_ZERO, 0, 0},   /* ¼ç£ºÄ¬ÈÏ ID 000 */
    {1, 0, XB_PULSE_ZERO, 0, 0},   /* Öâ£ºÄ¬ÈÏ ID 001 */
    {2, 0, XB_PULSE_ZERO, 0, 0},   /* ¼Ğ×¦£ºÄ¬ÈÏ ID 002 */
};

/* UART4 ÄÚºËÊ±ÖÓÔ´Ñ¡ PCLK1 ²¢³õÊ¼»¯ÍâÉè£¨CubeMX Î´Éú³É UART4£¬´Ë´¦×ÔÀí£© */
static void xb_uart4_hw_init(void)
{
    RCC_PeriphCLKInitTypeDef clk = {0};
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_UART4_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    clk.PeriphClockSelection = RCC_PERIPHCLK_UART4;
    clk.Usart16ClockSelection = RCC_USART16910CLKSOURCE_D2PCLK2;   /* ä¸ usart.c ç°æœ‰ UART æ—¶é’ŸåŸŸä¸€è‡´ */
    HAL_RCCEx_PeriphCLKConfig(&clk);

    /* µ¥Ïß°ëË«¹¤Ö»ĞèÅä TX ½Å PC10£¨AF8£©£¬RX ÄÚ²¿¾­ HDSEL ¸´ÓÃÍ¬Ò»½Å */
    gpio.Pin       = GPIO_PIN_10;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOC, &gpio);

    xb_huart.Instance                    = UART4;
    xb_huart.Init.BaudRate               = XB_BAUDRATE;
    xb_huart.Init.WordLength             = UART_WORDLENGTH_8B;
    xb_huart.Init.StopBits               = UART_STOPBITS_1;
    xb_huart.Init.Parity                 = UART_PARITY_NONE;
    xb_huart.Init.Mode                   = UART_MODE_TX_RX;
    xb_huart.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    xb_huart.Init.OverSampling           = UART_OVERSAMPLING_16;
    xb_huart.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    xb_huart.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
    xb_huart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_HalfDuplex_Init(&xb_huart) != HAL_OK) {
        return;   /* Ê§°ÜÔòÇı¶¯²»¹¤×÷£¬Éè±¸±í±£³Ö offline */
    }

    HAL_NVIC_SetPriority(UART4_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);

    xb_inited = 1;
}

/* ---- ÖĞ¶ÏÈë¿Ú£ºstartup ÏòÁ¿±í°´·ûºÅÃûÁ´½Ó ---- */
void UART4_IRQHandler(void)
{
    HAL_UART_IRQHandler(&xb_huart);
}

/* ---- RX£ºµ¥×Ö½ÚÖĞ¶ÏÈë»·ĞÎ»º³å£¨È«¹¤³ÌÎŞÆäËû RxCpltCallback£¬´Ë´¦ÎªÎ¨Ò»¶¨Òå£© ---- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4) {
        uint8_t next = (uint8_t)((rx_head + 1) % sizeof(rx_buf));
        if (next != rx_tail) {              /* ÂúÔò¶ªÆú×îĞÂ×Ö½Ú */
            rx_buf[rx_head] = rx_byte;
            rx_head = next;
        }
        HAL_UART_Receive_IT(&xb_huart, (uint8_t *)&rx_byte, 1);
    }
}

/* ---- Ö¡·¢ËÍ£º×èÈûÊ½¶ÌÖ¡£»·¢ËÍÆÚ¼ä¹Ø RE ÆÁ±Îµ¥Ïß»ØÉù ---- */
static uint8_t xb_send_frame(const char *frame)
{
    if (!xb_inited) return 2;

    (void)HAL_HalfDuplex_EnableTransmitter(&xb_huart);   /* ç½®TEæ¸…REï¼šå‘é€æœŸå±è”½å•çº¿å›å£° */
    HAL_UART_Transmit(&xb_huart, (uint8_t *)frame, (uint16_t)strlen(frame), 10);
    (void)HAL_HalfDuplex_EnableReceiver(&xb_huart);     /* æ¢å¤æ¥æ”¶ */
    return 0;
}

/* ---- Ó¦´ğ½âÎö£ºÉ¨Ãè #...! Ö¡ ----
 * Î»ÖÃÓ¦´ğ #000P1500!£»ÎÂ¶ÈµçÑ¹ #000T32V7!£»ÆäÓà(#OK!/#000PZServo..G!)ºöÂÔ */
static void xb_parse_frames(void)
{
    while (rx_tail != rx_head) {
        if (rx_buf[rx_tail] != '#') { rx_tail = (uint8_t)((rx_tail + 1) % sizeof(rx_buf)); continue; }
        /* ÊÕ¼¯Ò»Ö¡£¨µ½ '!' »ò»º³å¾¡Í·£© */
        char    f[24];
        uint8_t len = 0;
        uint8_t idx = rx_tail;
        do {
            f[len++] = (char)rx_buf[idx];
            idx = (uint8_t)((idx + 1) % sizeof(rx_buf));
            if (len >= sizeof(f) - 1) break;
        } while (f[len - 1] != '!' && idx != rx_head);
        rx_tail = idx;   /* Ïû·ÑµôÕâÒ»Ö¡£¨º¬Î´±ÕºÏ²ĞÖ¡£¬·À¿¨ËÀ£© */
        if (f[len - 1] != '!') continue;

        f[len] = '\0';
        /* #dddPdddd! Î»ÖÃÓ¦´ğ£¨PRAD£© */
        uint8_t  id;
        uint16_t pulse;
        if (len == 11 && f[4] == 'P' &&
            sscanf(f + 1, "%3hhuP%4hu", &id, &pulse) == 2) {
            for (uint8_t j = 0; j < XB_JOINT_NUM; j++) {
                if (xb_servo[j].id == id) {
                    xb_servo[j].pulse  = pulse;
                    xb_servo[j].online = 1;
                }
            }
            continue;
        }
        /* #dddTddVd! ÎÂ¶ÈµçÑ¹Ó¦´ğ£¨PRTV£© */
        int16_t id16, temp, volt;
        if (len == 10 && f[4] == 'T' &&
            sscanf(f + 1, "%3hdT%2hdV%1hd", &id16, &temp, &volt) == 3) {
            for (uint8_t j = 0; j < XB_JOINT_NUM; j++) {
                if (xb_servo[j].id == (uint8_t)id16) {
                    xb_servo[j].temp_c = temp;
                    xb_servo[j].volt_v = volt;
                    xb_servo[j].online = 1;
                }
            }
        }
    }
}

/* ================= ¶ÔÍâ½Ó¿Ú ================= */

void xb_servo_init(void)
{
    xb_uart4_hw_init();
    if (xb_inited) {
        HAL_UART_Receive_IT(&xb_huart, (uint8_t *)&rx_byte, 1);
    }
}

uint8_t xb_servo_set_pos(uint8_t joint, uint16_t pulse, uint16_t time_ms)
{
    if (joint >= XB_JOINT_NUM) return 1;
    if (pulse < XB_PULSE_MIN || pulse > XB_PULSE_MAX) return 1;
    if (time_ms > XB_TIME_MAX_MS) time_ms = XB_TIME_MAX_MS;

    char frame[24];
    snprintf(frame, sizeof(frame), "#%03uP%04uT%04u!", xb_servo[joint].id, pulse, time_ms);
    return xb_send_frame(frame);
}

uint8_t xb_servo_release(uint8_t joint)
{
    if (joint >= XB_JOINT_NUM) return 1;
    char frame[16];
    snprintf(frame, sizeof(frame), "#%03uPULK!", xb_servo[joint].id);
    return xb_send_frame(frame);
}

uint8_t xb_servo_request_pos(uint8_t joint)
{
    if (joint >= XB_JOINT_NUM) return 1;
    char frame[16];
    snprintf(frame, sizeof(frame), "#%03uPRAD!", xb_servo[joint].id);
    return xb_send_frame(frame);
}

uint8_t xb_servo_ping(uint8_t joint)
{
    if (joint >= XB_JOINT_NUM) return 1;
    char frame[16];
    snprintf(frame, sizeof(frame), "#%03uPVER!", xb_servo[joint].id);
    if (xb_send_frame(frame) != 0) return 0;
    osDelay(20);                    /* Ó¦´ğ¿ÕÖĞÊ±¼ä ~1.5ms + ¶æ»ú´¦Àí */
    uint8_t head_before = rx_head;
    xb_parse_frames();
    return (head_before != rx_head) ? 1 : 0;   /* ÊÕµ½ÈÎºÎĞÂÖ¡¼´ÈÏÎªÔÚÏß */
}

void xb_servo_poll(void)
{
    xb_parse_frames();
}

uint16_t xb_servo_get_pulse(uint8_t joint)
{
    if (joint >= XB_JOINT_NUM) return 0;
    return xb_servo[joint].pulse;
}
