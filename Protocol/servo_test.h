/**
  * @file servo_test.h
  * @brief 调试控制台接口
  * @note 输入来自 USART10 中断和板载 USB 虚拟串口，命令由 arm_task 顺序执行。
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
void servo_test_rx_byte(char ch);     /* 第二路输入（USB CDC 中断）逐字节调用 */
void servo_test_process(void);
void servo_test_poll(void);
void servo_test_set_config_allowed(uint8_t allowed);
uint8_t servo_test_config_active(void);

#endif
