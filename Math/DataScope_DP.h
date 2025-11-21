#ifndef __DATA_PRTOCOL_H
#define __DATA_PRTOCOL_H

#define VOFA +

#include "stm32h7xx_hal.h"

void DataWave(UART_HandleTypeDef* huart);
void DataWavePkg(void);
void DataScope_Get_Channel_Data(float Data);


#endif



