#ifndef FAKE_STATUS_TASK_H
#define FAKE_STATUS_TASK_H
#include <stdint.h>
#define WDG_BIT_BSP_REMOTE   (1UL << 12)
void wdg_user_set_bit(uint32_t bit);
#endif
