/**
  ******************************************************************************
  * @file    usart_comm.c
  * @author  ZZY 1950828677@qq.com
  * @brief   1.2.3.6.9.10
  * @date    2025-10
  * @version 1.0
  * @
  ==============================================================================
*/

///* Includes ------------------------------------------------------------------*/
//#include "usart_comm.h"
//#include "status_task.h"
/* Private define ------------------------------------------------------------*/

///* Private variables ---------------------------------------------------------*/
////TX
//uint8_t usb_tx_buf[128];



////RX

////uint8_t Uart1_Rx_Buff[USART1_RX_BUFFER_SIZE];
////uint8_t Uart2_Rx_Buff[USART2_RX_BUFFER_SIZE];     /*!< Receive buffer for Uart2 */
////uint8_t Uart3_Rx_Buff[USART3_RX_BUFFER_SIZE];     /*!< Receive buffer for Uart3 */
////uint8_t Uart4_Rx_Buff[USART4_RX_BUFFER_SIZE];     /*!< Receive buffer for Uart4 */
////uint8_t Uart10_Rx_Buff[USART5_RX_BUFFER_SIZE];     /*!< Receive buffer for Uart5 */
////uint8_t Uart6_Rx_Buff[USART6_RX_BUFFER_SIZE];     /*!< Receive buffer for Uart6 */
///*为什么从上面这种变成下面这种呢？ 是因为上面那种使用keil的debug——watch的时候会出现看不到数据的情况，拷打了一下ai发现只是watch不到，数据是有的。为了防止后续调试的时候懵了，故解决这个看不到的问题如下，具体情况请见开发日志，大概在2025年10月18日*/
//volatile uint8_t Uart1_Rx_Buff[USART1_RX_BUFFER_SIZE] __attribute__((section(".RAM_D1")));
//volatile uint8_t Uart2_Rx_Buff[USART2_RX_BUFFER_SIZE] __attribute__((section(".RAM_D1")));
//volatile uint8_t Uart3_Rx_Buff[USART3_RX_BUFFER_SIZE] __attribute__((section(".RAM_D1")));
//volatile uint8_t Uart6_Rx_Buff[USART6_RX_BUFFER_SIZE] __attribute__((section(".RAM_D1")));
//volatile uint8_t Uart10_Rx_Buff[USART10_RX_BUFFER_SIZE] __attribute__((section(".RAM_D1")));



//uint32_t i;
//volatile  uint32_t j;
///* Private type --------------------------------------------------------------*/

///* Private function declarations ---------------------------------------------*/
//static uint32_t User_USART1_RxCpltCallback(uint8_t *buf, uint16_t len);
//static uint32_t User_USART2_RxCpltCallback(uint8_t *buf, uint16_t len);
//static uint32_t User_USART3_RxCpltCallback(uint8_t *buf, uint16_t len);
//static uint32_t User_USART6_RxCpltCallback(uint8_t *buf, uint16_t len);
//static uint32_t User_USART10_RxCpltCallback(uint8_t *buf, uint16_t len);

///* function prototypes -------------------------------------------------------*/


//void comm_usart_init(void)
//{
//    bsp_user_usart_Init(&huart1,(uint8_t *)Uart1_Rx_Buff,USART1_RX_BUFFER_SIZE,User_USART1_RxCpltCallback);
//    bsp_user_usart_Init(&huart2, (uint8_t *)Uart2_Rx_Buff, USART2_RX_BUFFER_SIZE, User_USART2_RxCpltCallback);
//    bsp_user_usart_Init(&huart6, (uint8_t *)Uart6_Rx_Buff, USART6_RX_BUFFER_SIZE, User_USART6_RxCpltCallback);
//    bsp_user_usart_Init(&huart10,(uint8_t *)Uart10_Rx_Buff,USART10_RX_BUFFER_SIZE,User_USART10_RxCpltCallback);
//	log_init(&DEBUG_HUART);
//}


///**
//* @brief  串口数据发送任务 
//* @param  None.
//* @return None.
//*/
//void Task_USART_Transmit(void *arg)
//{
//  /* Cache for Task */
//  /* Pre-Load for task */
//  
//  /* Infinite loop */
//  
//  for(;;)
//  {
//    /* USART1 Send Port */
////    if(xQueueReceive(CAN1_TxPort,&CAN_TxMsg,portMAX_DELAY) == pdPASS)
////    {
////      do{
////        free_can_mailbox = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
////      }while(free_can_mailbox == 0);
////      bsp_can_std_send(&bsp_user_can1,CAN_TxMsg.ID,CAN_TxMsg.Data);
////    }

//    /* USART1 Send Port */
////    if(xQueueReceive(CAN1_TxPort,&CAN_TxMsg,portMAX_DELAY) == pdPASS)
////    {
////      do{
////        free_can_mailbox = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
////      }while(free_can_mailbox == 0);
////      bsp_can_std_send(&bsp_user_can1,CAN_TxMsg.ID,CAN_TxMsg.Data);
////    }

//    /* USART1 Send Port */
////    if(xQueueReceive(CAN1_TxPort,&CAN_TxMsg,portMAX_DELAY) == pdPASS)
////    {
////      do{
////        free_can_mailbox = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
////      }while(free_can_mailbox == 0);
////      bsp_can_std_send(&bsp_user_can1,CAN_TxMsg.ID,CAN_TxMsg.Data);
////    }

//    /* USART1 Send Port */
////    if(xQueueReceive(CAN1_TxPort,&CAN_TxMsg,portMAX_DELAY) == pdPASS)
////    {
////      do{
////        free_can_mailbox = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
////      }while(free_can_mailbox == 0);
////      bsp_can_std_send(&bsp_user_can1,CAN_TxMsg.ID,CAN_TxMsg.Data);
////    }

//    /* USART1 Send Port */
////    if(xQueueReceive(CAN1_TxPort,&CAN_TxMsg,portMAX_DELAY) == pdPASS)
////    {
////      do{
////        free_can_mailbox = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
////      }while(free_can_mailbox == 0);
////      bsp_can_std_send(&bsp_user_can1,CAN_TxMsg.ID,CAN_TxMsg.Data);
////    }



//  }
//}


///*-----------------------------USART_RxCpltCallback-----------------------------------*/
//static uint32_t User_USART1_RxCpltCallback(uint8_t *buf, uint16_t len)
//{
//	wdg_user_set_bit(WDG_BIT_BSP_REMOTE);
//	rc_callback_handler(&rc,buf);   

//	memset(buf,0,len);  


//	return 0;
//}

//static uint32_t User_USART2_RxCpltCallback(uint8_t *buf, uint16_t len)
//{
//	Judge_Read_Data(buf);
//    memset(buf, 0, len);
//    return 0;
//}

//static uint32_t User_USART6_RxCpltCallback(uint8_t *buf, uint16_t len)
//{
//    memset(buf, 0, len);
//    return 0;
//}


///**
//* @brief  USART10的中断回调函数     U10最开始计划是给视觉的，现改视觉通信为usb
//* @param  uint8_t *buf  接收缓冲数组指针
//* @param  uint16_t len  接收缓冲数组长度
//* @retval None
//*/
//static uint32_t User_USART10_RxCpltCallback(uint8_t *buf, uint16_t len)
//{
//	vision.rx_cnt++;
//   vision_data_handler(buf);   
//   memset(buf, 0, len);
//	return 0;
//}

/*------------------------------------不同版本串口接收分界线-----------------------------*/
#include "usart_comm.h"
#include "status_task.h"
uint8_t	dma_judge_buf[DMA_JUDGE_LEN];
uint8_t dma_dbus_buf[DMA_DBUS_LEN];
uint8_t dma_new_rc_buf[DMA_NEW_RC_LEN];
uint8_t dma_tuchuan_buf[DMA_TUCHUAN_LEN];
uint8_t dma_vision_buf[DMA_VISION_LEN];
//uint8_t dma_A3_buf[DMA_A3_LEN];

void USER_UART_IDLECallback(UART_HandleTypeDef *huart)
{
		if(huart->Instance== USART1)			//DBUS串口
    {
				wdg_user_set_bit(WDG_BIT_BSP_REMOTE);
        rc_callback_handler(&rc,dma_dbus_buf);
				memset(dma_dbus_buf, 0, DMA_DBUS_LEN);
        HAL_UART_Receive_DMA(huart, dma_dbus_buf, DMA_DBUS_LEN);
    }
		
	 else if(huart->Instance==USART3)    
		{
//			  wdg_user_set_bit(WDG_BIT_BSP_REMOTE);
//				new_rc_callback_handler(&new_rc,dma_new_rc_buf);
//				memset(dma_new_rc_buf, 0, DMA_NEW_RC_LEN);
//				HAL_UART_Receive_DMA(huart, dma_new_rc_buf, DMA_NEW_RC_LEN);
		}
		
				
	 else if(huart->Instance==UART5)    //新图传串口
		{
////			  wdg_user_set_bit(WDG_BIT_BSP_REMOTE);
////        new_rc_callback_handler(&new_rc_data,dma_new_rc_buf);
//				memset(dma_new_rc_buf, 0, DMA_NEW_RC_LEN);
//    HAL_UART_Receive_DMA(&TUCHUAN_HUART, dma_tuchuan_buf, DMA_TUCHUAN_LEN);
		}
		
		

    else if(huart->Instance== USART2)	//JUDGE串口
    {
        Judge_Read_Data(dma_judge_buf);
			  memset(dma_judge_buf, 0, DMA_JUDGE_LEN);
        HAL_UART_Receive_DMA(huart, dma_judge_buf, DMA_JUDGE_LEN);
    }
   
	
    else if(huart->Instance == USART10)	//VISION串口
    {
		  	vision.rx_cnt++;
        vision_data_handler(dma_vision_buf);
			  memset(dma_vision_buf, 0, DMA_VISION_LEN);
        HAL_UART_Receive_DMA(huart,dma_vision_buf,DMA_VISION_LEN);
    }
}

/**
  * @brief 串口空闲中断   注：需在it.c中每个串口的中断中调用该函数
  * @param UART_HandleTypeDef *huart
  * @retval 无
  */
void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) //判断是否是空闲中断
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart);                     //清除空闲中断标志（否则会一直不断进入中断）
			HAL_UART_AbortReceive(huart);						//仅关闭接收中断
//        HAL_UART_DMAStop(huart);															//停止本次DMA运输
        USER_UART_IDLECallback(huart);                     //调用串口功能回调函数
    }
}


/**
* @brief  串口初始化:使能串口空闲中断,开启串口DMA接收
* @param  无
* @retval 无
*/
void USER_UART_Init()
{
    __HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);
    __HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&DBUS_HUART, dma_dbus_buf, DMA_DBUS_LEN);
    
	  __HAL_UART_CLEAR_IDLEFLAG(&NEW_RC_HUART);
    __HAL_UART_ENABLE_IT(&NEW_RC_HUART, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&NEW_RC_HUART, dma_new_rc_buf, DMA_NEW_RC_LEN);//图传链路
	
//		  __HAL_UART_CLEAR_IDLEFLAG(&TUCHUAN_HUART);
//    __HAL_UART_ENABLE_IT(&TUCHUAN_HUART, UART_IT_IDLE);
//    HAL_UART_Receive_DMA(&TUCHUAN_HUART, dma_tuchuan_buf, DMA_TUCHUAN_LEN);
	
    __HAL_UART_CLEAR_IDLEFLAG(&JUDGE_HUART);
    __HAL_UART_ENABLE_IT(&JUDGE_HUART, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&JUDGE_HUART, dma_judge_buf, DMA_JUDGE_LEN);

    __HAL_UART_CLEAR_IDLEFLAG(&VISION_HUART);
    __HAL_UART_ENABLE_IT(&VISION_HUART, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&VISION_HUART,dma_vision_buf,DMA_VISION_LEN);
	log_init(&DEBUG_HUART);
}

