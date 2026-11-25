/* 主机测试台用的假头文件：只提供固件源文件会用到的那几个名字。 */
#ifndef FAKE_MAIN_H
#define FAKE_MAIN_H

#include <stdint.h>
#include <stddef.h>

typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;

#define GPIOA                 ((void *)0)
#define GPIO_PIN_15           0x8000U
#define GPIO_MODE_INPUT       0U
#define GPIO_PULLUP           1U
#define GPIO_NOPULL           0U
#define GPIO_SPEED_FREQ_LOW   0U

typedef struct { uint32_t Pin, Mode, Pull, Speed; } GPIO_InitTypeDef;

uint32_t      HAL_GetTick(void);
GPIO_PinState HAL_GPIO_ReadPin(void *port, uint16_t pin);
void          HAL_GPIO_Init(void *port, GPIO_InitTypeDef *g);
void          Error_Handler(void);

#define __HAL_RCC_GPIOA_CLK_ENABLE()   do { } while (0)

/* ---- 关中断 ---- */
extern uint32_t g_sim_primask;
#define __get_PRIMASK()        (g_sim_primask)
#define __disable_irq()        (g_sim_primask = 1U)
#define __set_PRIMASK(v)       (g_sim_primask = (uint32_t)(v))

/* ---- 串口寄存器的模拟 ----
   run.py 会把固件里的 UART5->ISR / UART5->RDR / UART5->ICR（USART10 同理）改写成下面三个函数调用
   （只改测试用的临时副本，不动固件文件）。uart = 5 或 10。 */
#define USART_ISR_RXNE_RXFNE  (1UL << 5)
#define USART_ISR_ORE         (1UL << 3)
#define USART_ISR_FE          (1UL << 1)
#define USART_ISR_NE          (1UL << 2)
#define USART_ISR_IDLE        (1UL << 4)
#define USART_ICR_IDLECF      (1UL << 4)
#define USART_ICR_ORECF       (1UL << 3)
#define USART_ICR_FECF        (1UL << 1)
#define USART_ICR_NECF        (1UL << 2)
#define UART_CLEAR_OREF       USART_ICR_ORECF
#define UART_CLEAR_FEF        USART_ICR_FECF
#define UART_CLEAR_NEF        USART_ICR_NECF
#define UART_IT_RXNE          (1UL << 5)
#define UART_IT_IDLE          (1UL << 4)

uint32_t            sim_uart_isr(int uart);          /* 当前标志 */
uint8_t             sim_uart_rdr(int uart);          /* 出队一个字节 */
volatile uint32_t  *sim_uart_icr(int uart);           /* 写 1 清对应标志 */
void                sim_uart_clear_flag(int uart, uint32_t flags);

typedef struct { int uart; } UART_HandleTypeDef;
extern UART_HandleTypeDef huart5;     /* 测试里先把 .uart 填成 5 */
extern UART_HandleTypeDef huart10;

#define __HAL_UART_CLEAR_FLAG(h, f)   sim_uart_clear_flag((h)->uart, (f))
#define __HAL_UART_DISABLE_IT(h, i)   ((void)(i))
#define __HAL_UART_ENABLE_IT(h, i)    ((void)(i))

#define USART10_IRQn   1
#define UART5_IRQn     2
void HAL_NVIC_SetPriority(int irq, uint32_t preempt, uint32_t sub);
void HAL_NVIC_EnableIRQ(int irq);

#endif
