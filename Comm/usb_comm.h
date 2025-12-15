/**
  ******************************************************************************
  * @file    
  * @author  ZZY 1950828677@qq.com
  * @brief   
  * @date    2025-12
  * @version 1.0
  * @
  ==============================================================================
*/

#ifndef __USART_COMM_H
#define __USART_COMM_H


#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"


/* Private macros ------------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/
void comm_usb_init(void);
void Task_USB_Transmit(void *arg);
#endif
