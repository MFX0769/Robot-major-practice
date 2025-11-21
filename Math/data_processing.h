#ifndef __DATA_PROCESSING_H
#define __DATA_PROCESSING_H

#ifdef  __DATA_PROCESSING_GLOBALS
#define __DATA_PROCESSING_EXT
#else
#define __DATA_PROCESSING_EXT extern
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#define ABS(x)		((x>0)? (x): (-x))

extern float circle_error(float set,float get,float circle_para);
extern void abs_limit(float *a, float ABS_MAX,float offset);
#endif

