/**
  * @file servo_test.h
  * @brief 舵机串口调试测试接口
  * @note 固定使用 U10/COM141, 命令由 arm_task 顺序执行。
  */
#ifndef SERVO_TEST_H
#define SERVO_TEST_H

#include <stdint.h>

#define SERVO_TEST_LINE_MAX 48U
#define SERVO_TEST_QUEUE_LEN 8U

typedef struct {
    char line[SERVO_TEST_LINE_MAX];
} servo_test_msg_t;

void servo_test_init(void);
void servo_test_rx_isr(void);
void servo_test_process(void);
void servo_test_poll(void);
void servo_test_set_config_allowed(uint8_t allowed);
uint8_t servo_test_config_active(void);

#endif
