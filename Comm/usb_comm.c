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
uint8_t usb_buf[30];
extern QueueHandle_t VISION_send_queue;
/* Private type --------------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/*-----------------------------CAN��ʼ����ʼ-----------------------------------*/
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
void Task_USB_Transmit(void *arg)
{
  /* Cache for Task */

  /* Pre-Load for task */
  
  /* Infinite loop */
  
  for(;;)
  {

   if(xQueueReceive(VISION_send_queue,usb_buf,portMAX_DELAY) == pdPASS)
   {
     CDC_Transmit_HS(usb_buf, 30);
   }
  }
}
