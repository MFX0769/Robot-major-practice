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
#include "Vision_Send_task.h"

/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint8_t usb_buf[30];
/* Private type --------------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/


void Task_Vision_Send(void const *argu)
{
    uint32_t wake_time = osKernelSysTick();

    for (;;)
    {
        taskENTER_CRITICAL();

        vision_output_data();

        taskEXIT_CRITICAL();

     osDelayUntil(&wake_time, 3);
    }
}
