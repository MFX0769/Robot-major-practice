#ifndef __PROT_NEW_RC_H__
#define __PROT_NEW_RC_H__


/* Includes ------------------------------------------------------------------*/
#include "stdbool.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"



/* Private macros ------------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
#define DMA_NEW_RC_LEN 21
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/


/* Exported variables --------------------------------------------*/


/* Exported function declarations --------------------------------------------*/


extern uint8_t dma_new_rc_buf[DMA_NEW_RC_LEN];


typedef  union
{
        uint8_t buff[DMA_NEW_RC_LEN];
         __packed struct
        {
						uint8_t sof_1;
						uint8_t sof_2;
						uint64_t ch_0:11;
						uint64_t ch_1:11;
						uint64_t ch_2:11;
						uint64_t ch_3:11;
						uint64_t mode_sw:2;
						uint64_t pause:1;
						uint64_t fn_1:1;
						uint64_t fn_2:1;
						uint64_t wheel:11;
						uint64_t trigger:1;

						int16_t mouse_x;
						int16_t mouse_y;
						int16_t mouse_z;
						uint8_t mouse_left:2;
						uint8_t mouse_right:2;
						uint8_t mouse_middle:2;
						uint16_t key;
						uint16_t crc16;
       }data;
} __attribute__((packed)) new_rc_data_t;


extern new_rc_data_t new_rc_data;

typedef enum
{
    MODE_C = 0,
    MODE_N = 1,
    MODE_S = 2,
}new_rc_mode_sw_e;//新遥控挡位切换开关  C:0 N:1 S:2

void new_rc_callback_handler(new_rc_data_t *new_rc_data ,uint8_t * buff);

#endif
