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

#ifndef __USB_COMM_H
#define __USB_COMM_H


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
extern uint8_t usb_buf[30];
/* Exported function declarations --------------------------------------------*/
void comm_usb_init(void);
void Task_USB_Transmit(void const *arg);

/* ---- unified USB tx queue (2026-11-22): single usb task owns CDC_Transmit_HS.
   All callers (vofa 1kHz / dbg_print / future) enqueue; usb_task drains. ---- */
#define USB_TX_MSG_DATA_LEN 120
typedef struct {
    uint16_t len;
    uint8_t  data[USB_TX_MSG_DATA_LEN];
} usb_tx_msg_t;

void usb_tx_send(const uint8_t *buf, uint16_t len);
void usb_tx_stats_get(uint32_t *enq, uint32_t *drop, uint32_t *tx, uint32_t *busy_drop);

#endif
