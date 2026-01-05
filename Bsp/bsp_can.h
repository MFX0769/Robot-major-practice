/**
  ******************************************************************************
  * @file   
  * @author 
  * @brief  
  ******************************************************************************
  * @attention
  *****************
*/

#ifndef __BSP_CAN_H
#define __BSP_CAN_H


#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/

#if defined(STM32F405xx) || defined(STM32F407xx)
	#include "stm32f4xx_hal.h"
#endif
#if defined(STM32F103xx)
	#include "stm32f1xx_hal.h"
#endif
#if defined(STM32H723xx)
#include "stm32h7xx.h"
#endif	

#include "fdcan.h"
#include <stdint.h>



/* Private macros ------------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
// typedef uint32_t (*can_call_back)(uint8_t can_id, uint32_t identifier, uint8_t* data,
//                                    uint32_t len);
                               

/* CAN过滤器过滤规则宏定义封装 为了尽量减少更换平台之后要改的部分，所以再封装一次，减少对"HAL"的使用 */
typedef enum{
      FLT_RANGE = 0x0000U,
      FLT_DUAL = 0x0001U,
      FLT_MASK = 0x0002U
}bsp_can_filter_type_t;

 typedef enum
 {
 	CAN_CHANNEL_1,
 	CAN_CHANNEL_2,
 	CAN_CHANNEL_3,
 	CAN_CHANNEL_NUM,
 }can_channel_e;

/*----------------------------------CAN过滤器规则配置相关-------------------------------------*/
// bsp_can_filter规则表，由上层使用者填写 。变长表头 + 
typedef struct {
    uint8_t  count;         /* 用户填：我将填写几条过滤规则 */
    uint16_t  data[30];     /* 3参数*10条=30。每 3 字节：type(1) + id1(2) + id2(2)  */
} bsp_can_filt_tbl_t;
/*----------------------------------CAN过滤器规则配置相关-------------------------------------*/



/*----------------------------------bsp层CAN结构体封装--------------------------------------*/
typedef struct CAN_TxMessage
{
  FDCAN_TxHeaderTypeDef  txHeader;
  uint8_t                tx_data[8];
}CAN_TxBuffer_t;

typedef struct CAN_RxMessage
{
  FDCAN_RxHeaderTypeDef  rxHeader;
  uint8_t                rx_data[8];
}CAN_RxBuffer_t;


typedef struct BSP_CanTypeDef_t {
    FDCAN_HandleTypeDef *hfdcan; 

    CAN_TxBuffer_t CAN_TxBuffer;

    CAN_RxBuffer_t CAN_RxBuffer;
    
} bsp_user_can_t;
/*----------------------------------bsp层CAN结构体封装--------------------------------------*/





//BSP层的CAN状态封装，留给中间层用
typedef enum{
	 CAN_OK = 0,
	 CAN_ERROR = 1,
	 CAN_LINE_BUSY = 2,
	 CAN_TIMEOUT = 3
}bsp_can_state_t;//具体数字和HAL_StatusTypeDef一样，但还是想封装一下方便移植



/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern bsp_user_can_t bsp_user_can1;
extern bsp_user_can_t bsp_user_can2;
extern bsp_user_can_t bsp_user_can3;


extern bsp_can_filt_tbl_t bsp_can1_filt_tbl;
extern bsp_can_filt_tbl_t bsp_can2_filt_tbl;
extern bsp_can_filt_tbl_t bsp_can3_filt_tbl;


/* Exported function declarations --------------------------------------------*/
void bsp_can_init(bsp_user_can_t *canx, void (*pFunc)(CAN_RxBuffer_t*));       //BSP层封装的CAN通讯总的初始化，应在协议初始化文件中调用此函数（调用前填写过滤规则表
// HAL_StatusTypeDef bsp_can_std_transmit(can_channel_e can_periph, uint32_t id, uint8_t *data);
HAL_StatusTypeDef bsp_can_std_transmit(can_channel_e can_periph, uint32_t id, uint8_t *data);


#endif
