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
static uint32_t s_enq, s_drop, s_tx_count, s_busy_drop;   /* 必须在 usb_tx_send 首次调用前定义 */

/* unified USB tx enqueue (2026-11-22): non-blocking, drop-on-full */
void usb_tx_send(const uint8_t *buf, uint16_t len)
{
    usb_tx_msg_t m;
    if (USB_tx_queue == NULL || buf == NULL || len == 0) return;
    if (len > USB_TX_MSG_DATA_LEN) len = USB_TX_MSG_DATA_LEN;
    m.len = len;
    memcpy(m.data, buf, len);
    if (xQueueSend(USB_tx_queue, &m, 0) == pdPASS) s_enq++;
    else s_drop++;                       /* full -> drop; count it so print overload is diagnosable */
}
extern USBD_HandleTypeDef hUsbDeviceHS;
static usb_tx_msg_t s_tx[2];
static uint8_t s_tx_idx;

void usb_tx_stats_get(uint32_t *enq, uint32_t *drop, uint32_t *tx, uint32_t *busy_drop)
{
    if (enq != NULL) *enq = s_enq;
    if (drop != NULL) *drop = s_drop;
    if (tx != NULL) *tx = s_tx_count;
    if (busy_drop != NULL) *busy_drop = s_busy_drop;
}

/* USB 还在读上一帧的缓冲区时返回 1 */
static uint8_t cdc_busy(void)
{
    USBD_CDC_HandleTypeDef *hcdc;
    if (hUsbDeviceHS.pClassData == NULL) return 0U;
    hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData;
    return (hcdc->TxState != 0U) ? 1U : 0U;
}

void Task_USB_Transmit(void const *arg)
{
  usb_tx_msg_t *msg;
  uint32_t w;
  (void)arg;
  for(;;)
  {
    if (USB_tx_queue == NULL)
    {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    /* CDC_Transmit_HS 是异步的：两个缓冲交替用，复用某个缓冲之前等它上一次发送完成 */
    msg = &s_tx[s_tx_idx];
    if (xQueueReceive(USB_tx_queue, msg, portMAX_DELAY) != pdPASS) continue;
    for (w = 0U; w < 20U && cdc_busy() != 0U; w++)
    {
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (cdc_busy() == 0U)
    {
      if (CDC_Transmit_HS(msg->data, msg->len) == USBD_OK) s_tx_count++;
      else s_busy_drop++;                 /* 未枚举/端点未就绪：明确计数，不假装发出去了 */
    }
    else s_busy_drop++;
    s_tx_idx ^= 1U;
  }
}
