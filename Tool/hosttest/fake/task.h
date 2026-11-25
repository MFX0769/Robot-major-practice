#ifndef FAKE_TASK_H
#define FAKE_TASK_H
#define taskENTER_CRITICAL()   do { } while (0)
#define taskEXIT_CRITICAL()    do { } while (0)
/* 真固件里 task.h 由 FreeRTOS 提供，队列接口跟着进来；控制台的行队列要用 */
#include "queue.h"
#endif
