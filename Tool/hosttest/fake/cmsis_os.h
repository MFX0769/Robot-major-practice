#ifndef FAKE_CMSIS_OS_H
#define FAKE_CMSIS_OS_H
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
uint32_t osKernelSysTick(void);
void     osDelay(uint32_t ms);
int      osDelayUntil(uint32_t *ticks, uint32_t ms);
#endif
