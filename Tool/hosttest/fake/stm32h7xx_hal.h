#ifndef FAKE_STM32H7XX_HAL_H
#define FAKE_STM32H7XX_HAL_H
#include "main.h"
/* armcc 的 __align(n)，主机编译器用 GNU 属性代替 */
#define __align(n)   __attribute__((aligned(n)))
#endif
