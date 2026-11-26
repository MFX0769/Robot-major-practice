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
/* PD14 -> UART9_RX（测距模块）用到的几个 GPIO 名字 */
#define GPIOD                 ((void *)1)
#define GPIO_PIN_14           0x4000U
#define GPIO_MODE_AF_PP       2U
#define GPIO_SPEED_FREQ_VERY_HIGH 3U
#define GPIO_AF11_UART9       11U
#define __HAL_RCC_GPIOD_CLK_ENABLE()   do { } while (0)
#define __HAL_RCC_UART9_CLK_ENABLE()   do { } while (0)

typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;

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

/* HAL 的串口初始化结构：tfmini_init() 会填一遍，这里只保证能编译 */
typedef struct {
    uint32_t BaudRate, WordLength, StopBits, Parity, Mode, HwFlowCtl, OverSampling, OneBitSampling, ClockPrescaler;
} UART_InitTypeDef;
typedef struct {
    int uart;                                  /* 模拟寄存器的编号 */
    void *Instance;
    UART_InitTypeDef Init;
    struct { uint32_t AdvFeatureInit; } AdvancedInit;
} UART_HandleTypeDef;
extern UART_HandleTypeDef huart5;     /* 测试里先把 .uart 填成 5 */
extern UART_HandleTypeDef huart10;
extern UART_HandleTypeDef huart9;     /* Protocol/tfmini.c 用 */
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *h);

#define __HAL_UART_CLEAR_FLAG(h, f)   sim_uart_clear_flag((h)->uart, (f))
#define __HAL_UART_DISABLE_IT(h, i)   ((void)(i))
#define __HAL_UART_ENABLE_IT(h, i)    ((void)(i))

#define USART10_IRQn   1
#define UART5_IRQn     2
#define UART9_IRQn     9
#define UART9          ((void *)9)
#define UART_WORDLENGTH_8B             0U
#define UART_STOPBITS_1                0U
#define UART_PARITY_NONE               0U
#define UART_MODE_RX                   1U
#define UART_HWCONTROL_NONE            0U
#define UART_OVERSAMPLING_16           0U
#define UART_ONE_BIT_SAMPLE_DISABLE    0U
#define UART_PRESCALER_DIV1            0U
#define UART_ADVFEATURE_NO_INIT        0U
void HAL_NVIC_SetPriority(int irq, uint32_t preempt, uint32_t sub);
void HAL_NVIC_EnableIRQ(int irq);

#endif
