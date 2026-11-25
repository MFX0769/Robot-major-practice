#ifndef FAKE_QUEUE_H
#define FAKE_QUEUE_H
#include <stdint.h>

typedef void *QueueHandle_t;
typedef int BaseType_t;

#define pdPASS        1
#define pdTRUE        1
#define pdFALSE       0
#define portMAX_DELAY 0xFFFFFFFFUL

QueueHandle_t xQueueCreate(uint32_t len, uint32_t item_size);
BaseType_t    xQueueSend(QueueHandle_t q, const void *item, uint32_t wait);
BaseType_t    xQueueSendFromISR(QueueHandle_t q, const void *item, void *woken);
BaseType_t    xQueueReceive(QueueHandle_t q, void *buf, uint32_t wait);

#endif
