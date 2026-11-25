/**
  * @file servo_test.c
  * @brief 舵机测试命令解析与执行
  * @note U10/COM141 命令入口; 仅 ping/read/setid/map, 不提供运动控制。
  *       ISR 只收整行入队, 任务上下文执行解析和串口事务。
  */
#include "servo_test.h"
#include "prot_xb_servo.h"
#include "prot_servo.h"
#include "arm_yaw_limit.h"
#include "prot_rm_motor.h"
#include "control_def.h"
#include "usart.h"
#include "dbg_print.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include <ctype.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static QueueHandle_t s_queue;
static char s_line[SERVO_TEST_LINE_MAX];
static uint8_t s_line_len;
static uint8_t s_config_allowed;
static uint8_t s_config_active;
static uint8_t s_yaw_monitor_active;
static uint8_t s_yaw_monitor_div;

static int32_t parse_signed(const char *s, int32_t *value)
{
    char *end = NULL;
    long v = strtol(s, &end, 10);
    if (s == end || *end != '\0' || v < INT32_MIN || v > INT32_MAX) {
        return 0;
    }
    *value = (int32_t)v;
    return 1;
}

static uint8_t parse_dec(const char *s, uint8_t *value)
{
    unsigned int v = 0;
    if (*s == '\0') return 0U;
    while (*s) {
        if (!isdigit((unsigned char)*s)) return 0U;
        v = v * 10U + (unsigned int)(*s - '0');
        if (v > 255U) return 0U;
        s++;
    }
    *value = (uint8_t)v;
    return 1U;
}

static void servo_test_parse_line(const char *line)
{
    char work[SERVO_TEST_LINE_MAX];
    char *token;
    uint8_t id, old_id, new_id;
    uint8_t is_yaw_cmd;
    snprintf(work, sizeof(work), "%s", line);
    token = strtok(work, " ");
    if (token == NULL) goto usage;
    is_yaw_cmd = (strcmp(token, "yaw") == 0) ? 1U : 0U;
    if (!is_yaw_cmd && strcmp(token, "servo") != 0) goto usage;
    token = strtok(NULL, " ");
    if (token == NULL) goto usage;

    if (is_yaw_cmd) {
        if (strcmp(token, "monitor") == 0) {
            if (strtok(NULL, " ") != NULL) goto usage;
            s_yaw_monitor_active = 1U;
            s_yaw_monitor_div = 0U;
            dbg_print("[YAW_LIMIT] monitor ON: zero torque, 20Hz\r\n");
        } else if (strcmp(token, "stop") == 0) {
            if (strtok(NULL, " ") != NULL) goto usage;
            s_yaw_monitor_active = 0U;
            dbg_print("[YAW_LIMIT] monitor OFF\r\n");
        } else if (strcmp(token, "set") == 0) {
            int32_t lower, upper;
            token = strtok(NULL, " ");
            if (token == NULL || !parse_signed(token, &lower)) goto usage;
            token = strtok(NULL, " ");
            if (token == NULL || !parse_signed(token, &upper)) goto usage;
            if (strtok(NULL, " ") != NULL) goto usage;
            if (arm_yaw_limit_set(lower, upper) != 0) {
                dbg_print("[YAW_LIMIT] set rejected: lower must be less than upper\r\n");
            } else {
                dbg_print("[YAW_LIMIT] set lower_ecd=%ld upper_ecd=%ld\r\n",
                          (long)arm_yaw_limit_config.lower_ecd,
                          (long)arm_yaw_limit_config.upper_ecd);
            }
        } else if (strcmp(token, "clear") == 0) {
            if (strtok(NULL, " ") != NULL) goto usage;
            arm_yaw_limit_clear();
            dbg_print("[YAW_LIMIT] limits cleared\r\n");
        } else {
            goto usage;
        }
        return;
    }

    if (strcmp(token, "ping") == 0) {
        token = strtok(NULL, " ");
        if (token == NULL || !parse_dec(token, &id) || id > 254U || strtok(NULL, " ") != NULL) goto usage;
        uint8_t rc = xb_servo_ping_raw(id);
        dbg_print("[SERVO_TEST] ping id=%u rc=%u reply=%s\r\n", id, rc, xb_servo_last_frame());
    } else if (strcmp(token, "read") == 0) {
        token = strtok(NULL, " ");
        if (token == NULL || !parse_dec(token, &id) || id > 254U || strtok(NULL, " ") != NULL) goto usage;
        uint16_t pulse = 0;
        uint8_t found = 0;
        for (uint8_t j = 0; j < XB_JOINT_NUM; j++) {
            if (xb_servo[j].id == id) {
                uint8_t rc = xb_servo_read_pos(j, &pulse);
                dbg_print("[SERVO_TEST] read id=%u rc=%u pulse=%u\r\n", id, rc, pulse);
                found = 1U;
                break;
            }
        }
        if (!found) {
            uint8_t read_id = 0;
            uint8_t rc = xb_servo_read_id_raw(id, &read_id);
            dbg_print("[SERVO_TEST] read id=%u rc=%u reply_id=%u\r\n", id, rc, read_id);
        }
    } else if (strcmp(token, "setid") == 0) {
        token = strtok(NULL, " ");
        if (token == NULL || !parse_dec(token, &old_id)) goto usage;
        token = strtok(NULL, " ");
        if (token == NULL || !parse_dec(token, &new_id)) goto usage;
        if (old_id > 254U || new_id > 254U || old_id == new_id || strtok(NULL, " ") != NULL) goto usage;
        if (!s_config_allowed || !s_config_active) {
            dbg_print("[SERVO_TEST] setid rejected: send 'servo config' first\r\n");
            return;
        }
        uint8_t rc = xb_servo_write_id_raw(old_id, new_id);
        dbg_print("[SERVO_TEST] setid old=%u new=%u rc=%u reply=%s\r\n",
                  old_id, new_id, rc, xb_servo_last_frame());
    } else if (strcmp(token, "bind") == 0) {
        uint8_t joint = 0;
        token = strtok(NULL, " ");
        if (token == NULL) goto usage;
        if (strcmp(token, "shoulder") == 0) joint = XB_SHOULDER;
        else if (strcmp(token, "elbow") == 0) joint = XB_ELBOW;
        else goto usage;
        token = strtok(NULL, " ");
        if (token == NULL || !parse_dec(token, &id) || id > 254U || strtok(NULL, " ") != NULL) goto usage;
        uint8_t rc = xb_servo_bind(joint, id);
        dbg_print("[SERVO_TEST] bind joint=%u id=%u rc=%u\r\n", joint, id, rc);
    } else if (strcmp(token, "config") == 0) {
        if (strtok(NULL, " ") != NULL) goto usage;
        servo_set_config_mode(1);
        s_config_active = 1;
        dbg_print("[SERVO_TEST] config mode ON: power held, motion inhibited, setid enabled\r\n");
    } else if (strcmp(token, "run") == 0) {
        if (strtok(NULL, " ") != NULL) goto usage;
        servo_set_config_mode(0);
        s_config_active = 0;
        dbg_print("[SERVO_TEST] config mode OFF\r\n");
    } else if (strcmp(token, "map") == 0) {
        if (strtok(NULL, " ") != NULL) goto usage;
        dbg_print("[SERVO_TEST] map shoulder=%u elbow=%u\r\n",
                  xb_servo[XB_SHOULDER].id, xb_servo[XB_ELBOW].id);
    } else {
        goto usage;
    }
    return;

usage:
    dbg_print("[SERVO_TEST] usage: servo ... | yaw monitor | yaw stop | yaw set <lower> <upper> | yaw clear\r\n");
}

void servo_test_rx_isr(void)
{
    uint32_t isr = USART10->ISR;
    if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE)) {
        __HAL_UART_CLEAR_FLAG(&huart10, UART_CLEAR_OREF);
        __HAL_UART_CLEAR_FLAG(&huart10, UART_CLEAR_FEF);
        __HAL_UART_CLEAR_FLAG(&huart10, UART_CLEAR_NEF);
    }
    while (USART10->ISR & USART_ISR_RXNE_RXFNE) {
        char ch = (char)USART10->RDR;
        if (ch == '\r' || ch == '\n') {
            if (s_line_len > 0U) {
                servo_test_msg_t msg;
                memcpy(msg.line, s_line, s_line_len);
                msg.line[s_line_len] = '\0';
                if (s_queue != NULL) {
                    (void)xQueueSendFromISR(s_queue, &msg, NULL);
                }
                s_line_len = 0;
            }
        } else if (s_line_len + 1U < sizeof(s_line)) {
            s_line[s_line_len++] = ch;
        } else {
            s_line_len = 0;
        }
    }
}

void servo_test_init(void)
{
    s_queue = xQueueCreate(SERVO_TEST_QUEUE_LEN, sizeof(servo_test_msg_t));
    s_line_len = 0;
    s_config_allowed = 0;
    s_config_active = 0;
    __HAL_UART_DISABLE_IT(&huart10, UART_IT_RXNE);
    while (USART10->ISR & USART_ISR_RXNE_RXFNE) (void)USART10->RDR;
    __HAL_UART_CLEAR_FLAG(&huart10, UART_CLEAR_OREF);
    __HAL_UART_CLEAR_FLAG(&huart10, UART_CLEAR_FEF);
    __HAL_UART_CLEAR_FLAG(&huart10, UART_CLEAR_NEF);
    HAL_NVIC_SetPriority(USART10_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART10_IRQn);
    __HAL_UART_ENABLE_IT(&huart10, UART_IT_RXNE);
}

void servo_test_process(void)
{
    servo_test_msg_t msg;
    if (s_queue == NULL || xQueueReceive(s_queue, &msg, 0) != pdTRUE) return;
    servo_test_parse_line(msg.line);
}

void servo_test_poll(void)
{
    if (!s_yaw_monitor_active) return;
    if (++s_yaw_monitor_div < (ARM_YAW_LIMIT_LOG_PERIOD_MS / ARM_TASK_PERIOD)) return;
    s_yaw_monitor_div = 0U;
    arm_yaw_limit_clamp_target(arm_yaw_feedback.total_count);
    dbg_print("[YAW_LIMIT] ecd=%ld total=%ld round=%ld angle=%.2f rpm=%d raw_i=%d temp=%u rx=%lu online=%u limits=%d side=%d\r\n",
              (long)arm_yaw_feedback.ecd,
              (long)arm_yaw_feedback.total_count,
              (long)arm_yaw_feedback.round_count,
              arm_yaw_feedback.angle_deg,
              arm_yaw_feedback.speed_rpm,
              arm_yaw_feedback.current_raw,
              (unsigned)arm_yaw_feedback.temperature,
              (unsigned long)arm_yaw_feedback.receive_count,
              (unsigned)arm_yaw_feedback.online,
              arm_yaw_limit_config.configured,
              (int)arm_yaw_limit_side());
}

void servo_test_set_config_allowed(uint8_t allowed)
{
    s_config_allowed = allowed ? 1U : 0U;
}

uint8_t servo_test_config_active(void)
{
    return s_config_active;
}
