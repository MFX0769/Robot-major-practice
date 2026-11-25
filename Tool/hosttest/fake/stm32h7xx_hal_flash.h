#ifndef FAKE_STM32H7XX_HAL_FLASH_H
#define FAKE_STM32H7XX_HAL_FLASH_H
#include "main.h"

#define FLASH_BANK_1                    0x01U
#define FLASH_SECTOR_6                  6U
#define FLASH_SECTOR_7                  7U
#define FLASH_FLAG_ALL_ERRORS_BANK1     0xFFFFFFFFU
#define FLASH_TYPEPROGRAM_FLASHWORD     0x01U
#define __HAL_FLASH_CLEAR_FLAG_BANK1(f) do { (void)(f); } while (0)

HAL_StatusTypeDef HAL_FLASH_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_Lock(void);
/* H7：一次编程一个 256 位（32 字节）的字，数据来自 data_addr 指向的 RAM */
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t addr, uint32_t data_addr);
#endif
