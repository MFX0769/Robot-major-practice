/**
  ******************************************************************************
  * @file    
  * @author  ZZY (1950828677@qq.com)
  * @brief   
  * @date    2025-12
  * @version 1.0
  * @note 

*/

/* Includes ------------------------------------------------------------------*/
#include "usb_comm.h"
#include <string.h>
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint8_t vision_usb_buf[30];//仅用作视觉数据在队列消息接收时的缓冲区
/* Private type --------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/ 
extern QueueHandle_t VISION_send_queue;
extern QueueHandle_t USB_tx_queue;

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/


/* Private functions -------------------------------------------------------*/

/**
* @brief  ��ʼ��CAN
* @param  bsp_user_can_t BSP���װ��CAN�˿�
* @retval None
*/
void comm_usb_init(void)
{

}
    

/*-----------------------------CAN��ʼ����ʼ-----------------------------------*/



/**
* @brief  CAN��������
* @param  None.
* @return None.
*/
/* 2026-11-22: single owner of CDC_Transmit_HS. vofa/dbg enqueue via usb_tx_send. */

/* unified USB tx enqueue (2026-11-22): non-blocking, drop-on-full */
void usb_tx_send(const uint8_t *buf, uint16_t len)
{
    usb_tx_msg_t m;
    if (USB_tx_queue == NULL || buf == NULL || len == 0) return;
    if (len > USB_TX_MSG_DATA_LEN) len = USB_TX_MSG_DATA_LEN;
    m.len = len;
    memcpy(m.data, buf, len);
    (void)xQueueSend(USB_tx_queue, &m, 0);   /* full -> drop (keeps 1kHz vofa alive) */
}
void Task_USB_Transmit(void const *arg)
{
  usb_tx_msg_t msg;
  (void)arg;
  for(;;)
  {
    if (USB_tx_queue != NULL &&
        xQueueReceive(USB_tx_queue, &msg, portMAX_DELAY) == pdPASS) {
      /* 第二出口：CDC 未枚举时返回失败直接丢，绝不阻塞 */
      if (msg.len > 0U) {
        (void)CDC_Transmit_HS(msg.data, msg.len);
      }
    }
  }
}
