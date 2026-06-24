#include "prot_8irtraking.h"
#include "usart.h"
#include "usart_comm.h"

static prot_8irtraking_data_t s_data;

void prot_8irtraking_init(void)
{
    memset(&s_data, 0, sizeof(s_data));
}

void prot_8irtraking_cmd_read(uint8_t analog_en, uint8_t digital_en)
{
    uint8_t buf[16];
    int n = snprintf((char*)buf, sizeof(buf), "$0,%d,%d#", analog_en ? 1 : 0, digital_en ? 1 : 0);
    if (n > 0)
        HAL_UART_Transmit(&TUCHUAN_HUART, buf, n, 100);
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

static void compute_position(void)
{
    int32_t sum = 0;
    int32_t weight = 0;
    for (int i = 0; i < PROT_8IR_NUM_SENSORS; i++) {
        if (s_data.digital[i]) {
            sum += (i - 3) * 25;
            weight++;
        }
    }
    if (weight == 0) {
        s_data.state = 0;
        s_data.position = 0;
    } else {
        s_data.state = 1;
        s_data.position = (int16_t)(sum / weight);
    }
}

void prot_8irtraking_on_receive(const uint8_t *buf, uint16_t len)
{
    if (!buf || len < 8) return;

    const uint8_t *p = buf;
    const uint8_t *end = buf + len;

    while (p < end - 7) {
        if (*p != '$') { p++; continue; }

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