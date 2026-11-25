#ifndef FAKE_STM32H7XX_HAL_FLASH_EX_H
#define FAKE_STM32H7XX_HAL_FLASH_EX_H
#include "stm32h7xx_hal_flash.h"

typedef struct {
    uint32_t TypeErase;
    uint32_t Banks;
    uint32_t Sector;
    uint32_t NbSectors;
    uint32_t VoltageRange;
} FLASH_EraseInitTypeDef;

#define FLASH_TYPEERASE_SECTORS   0x00U
#define FLASH_VOLTAGE_RANGE_4     4U

HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *e, uint32_t *sector_error);
#endif
