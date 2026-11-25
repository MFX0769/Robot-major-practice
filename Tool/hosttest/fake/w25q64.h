#ifndef FAKE_W25Q64_H
#define FAKE_W25Q64_H
#include "main.h"
#define OSPI_W25Qxx_OK        0
int8_t   OSPI_W25Qxx_BlockErase_32K(uint32_t addr);
int8_t   OSPI_W25Qxx_WriteBuffer(uint8_t *buf, uint32_t addr, uint32_t size);
int8_t   OSPI_W25Qxx_ReadBuffer(uint8_t *buf, uint32_t addr, uint32_t size);
uint32_t OSPI_W25Qxx_ReadID(void);
#endif
