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
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint8_t vision_usb_buf[30];//仅用作视觉数据在队列消息接收时的缓冲区
/* Private type --------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/ 
extern QueueHandle_t VISION_send_queue;

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
void Task_USB_Transmit(void const *arg)
{
  /* Cache for Task */

  /* Pre-Load for task */
  
  /* Infinite loop */
  
  for(;;)
  {

   if(xQueueReceive(VISION_send_queue,vision_usb_buf,portMAX_DELAY) == pdPASS)
   {
     CDC_Transmit_HS(vision_usb_buf, 44);
   }
  }
}
