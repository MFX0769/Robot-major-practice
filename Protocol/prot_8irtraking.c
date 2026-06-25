#include "prot_8irtraking.h"
#include "usart.h"
#include "usart_comm.h"

prot_8irtraking_data_t s_data;  /* 改成全局变量(去掉static)，方便Keil watch实时刷新查看，参考wbus_rc的做法 */

void prot_8irtraking_init(void)
{
    memset(&s_data, 0, sizeof(s_data));
}

int prot_8irtraking_cmd_read(uint8_t analog_en, uint8_t digital_en)
{
    uint8_t buf[16];
    int n = snprintf((char*)buf, sizeof(buf), "$0,%d,%d#", analog_en ? 1 : 0, digital_en ? 1 : 0);
    if (n > 0)
        return (int)HAL_UART_Transmit(&TUCHUAN_HUART, buf, n, 100);
    return -1;
}

static int parse_value(const char *p, const char *key, int *val)
{
    const char *k = strstr(p, key);
    if (!k) return -1;
    k += 3;
    *val = 0;
    while (*k >= '0' && *k <= '9') {
        *val = *val * 10 + (*k - '0');
        k++;
    }
    return 0;
}

/* 搬运自RoboticMajor_Pratice_code的Application/irtraking_task.c：等距对称权重+加权平均+归一化到±100。
   实际只有digital[1]~digital[6](X2~X7)接了传感器，digital[0]/[7](X1/X8)未接，权重设0占位 */ 	
static void compute_position(void)
{
    static const float weights[8] = {0.0f, -4.0f, -2.0f, -0.5f, 0.5f, 2.0f, 4.0f, 0.0f};
    float weighted_sum = 0.0f;
    float sensor_sum = 0.0f;

    for (int i = 1; i <= 6; i++) {
        /* 实测极性：digital=0是黑线，digital=1是白色/没在线上(2026-06-25确认)，这里取反才是"压在线上" */
        if (!s_data.digital[i]) {
            weighted_sum += weights[i];
            sensor_sum += 1.0f;
        }
    }

    if (sensor_sum < 0.5f) {
        s_data.state = 0;
        s_data.position = 0;
    } else {
        s_data.state = 1;
        s_data.position = (int16_t)(weighted_sum / sensor_sum * 100.0f / 2.5f);
    }
}

/* 改用纯GPIO直接读数字量(手册方式1)，不再走USART协议。
   实测接线(X7->X2，2026-06-25)：
   X7=U2_T(PA2)  X6=U2_R(PA3)  X5=U3_T(PB10)
   X4=U3_R(PB11) X3=U10_R(PE2) X2=U10_T(PE3)  X1/X8未接 */
void prot_8irtraking_poll_gpio(void)
{
    s_data.digital[1] = (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3)  == GPIO_PIN_SET) ? 1 : 0;  /* X2 */
    s_data.digital[2] = (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2)  == GPIO_PIN_SET) ? 1 : 0;  /* X3 */
    s_data.digital[3] = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_SET) ? 1 : 0;  /* X4 */
    s_data.digital[4] = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET) ? 1 : 0;  /* X5 */
    s_data.digital[5] = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)  == GPIO_PIN_SET) ? 1 : 0;  /* X6 */
    s_data.digital[6] = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)  == GPIO_PIN_SET) ? 1 : 0;  /* X7 */

    compute_position();
}

void prot_8irtraking_on_receive(const uint8_t *buf, uint16_t len)
{
    s_data.rx_cnt++;

    if (!buf || len < 8) return;

    const uint8_t *p = buf;
    const uint8_t *end = buf + len;

    while (p < end - 7) {
        if (*p != '$') 
				{ 
					p++; 	
					continue; 
				}

        const uint8_t *tail = (const uint8_t*)memchr(p + 1, '#', (size_t)(end - p - 1));
        if (!tail) break;

        char frame[128];
        uint16_t flen = (uint16_t)(tail - p + 1);
        if (flen >= sizeof(frame)) { p = tail + 1; continue; }
        memcpy(frame, p, flen);
        frame[flen] = '\0';

        if (frame[1] == 'D') {
            for (int i = 0; i < PROT_8IR_NUM_SENSORS; i++) {
                char key[4];
                snprintf(key, sizeof(key), "x%d", i + 1);
                int v = 0;
                if (parse_value(frame, key, &v) == 0)
                    s_data.digital[i] = (uint8_t)(v ? 1 : 0);
            }
            compute_position();
        }
        else if (frame[1] == 'A') {
            for (int i = 0; i < PROT_8IR_NUM_SENSORS; i++) {
                char key[4];
                snprintf(key, sizeof(key), "x%d", i + 1);
                int v = 0;
                if (parse_value(frame, key, &v) == 0)
                    s_data.analog[i] = (uint16_t)v;
            }
        }

        p = tail + 1;
    }
}

const prot_8irtraking_data_t *prot_8irtraking_get_data(void)
{
    return &s_data;
}