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
///*Ϊʲô���������ֱ�����������أ� ����Ϊ��������ʹ��keil��debug����watch��ʱ�����ֿ��������ݵ������������һ��ai����ֻ��watch�������������еġ�Ϊ�˷�ֹ�������Ե�ʱ�����ˣ��ʽ��������������������£�����������������־�������2025��10��18��*/
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
//* @brief  �������ݷ������� 
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
//* @brief  USART10���жϻص�����     U10�ʼ�ƻ��Ǹ��Ӿ��ģ��ָ��Ӿ�ͨ��Ϊusb
//* @param  uint8_t *buf  ���ջ�������ָ��
//* @param  uint16_t len  ���ջ������鳤��
//* @retval None
//*/
//static uint32_t User_USART10_RxCpltCallback(uint8_t *buf, uint16_t len)
//{
//	vision.rx_cnt++;
//   vision_data_handler(buf);   
//   memset(buf, 0, len);
//	return 0;
//}

/*------------------------------------串口收发接口-----------------------------*/
#include "usart_comm.h"
#include "status_task.h"
#include "prot_sbus.h"
#include "prot_8irtraking.h"

uint8_t dma_dbus_buf[DMA_DBUS_LEN];
uint8_t dma_sbus_buf[DMA_SBUS_LEN];  /* SBUS遥控器，跟dma_dbus_buf是两个东西，不要混用 */
uint8_t dma_new_rc_buf[VTM_DATA_LEN];
uint8_t dma_vision_buf[DMA_VISION_LEN];
uint8_t dma_8ir_buf[30];  /* 8路灰度传感器缓冲区 */

void USER_UART_IDLECallback(UART_HandleTypeDef *huart)
{
		if(huart->Instance== USART1)			//SBUS遥控器
    {
				wdg_user_set_bit(WDG_BIT_BSP_REMOTE);
        sbus_callback_handler(dma_sbus_buf, DMA_SBUS_LEN);
				memset(dma_sbus_buf, 0, DMA_SBUS_LEN);
        HAL_UART_Receive_DMA(huart, dma_sbus_buf, DMA_SBUS_LEN);
    }
		
	 else if(huart->Instance==UART9)    
		{
			  wdg_user_set_bit(WDG_BIT_BSP_REMOTE);
				vtm_callback_handler(dma_new_rc_buf);
				memset(dma_new_rc_buf, 0, VTM_DATA_LEN);
				HAL_UART_Receive_DMA(huart, dma_new_rc_buf, VTM_DATA_LEN);
		 
		}
		
				
    else if(huart->Instance== USART10)    //循迹模块改接U10,8路灰度传感器
		{
//				prot_8irtraking_on_receive(dma_8ir_buf, 30);
//				memset(dma_8ir_buf, 0, sizeof(dma_8ir_buf));
				HAL_UART_Receive_DMA(huart, dma_8ir_buf, 30);
		}
   
	
    else if(huart->Instance == USART6)	//视觉改接U6,VISION����
    {
		  	vision.rx_cnt++;
		uint32_t rx_len = DMA_VISION_LEN - __HAL_DMA_GET_COUNTER(huart->hdmarx);
        vision_data_handler(dma_vision_buf,rx_len);
			  memset(dma_vision_buf, 0, DMA_VISION_LEN);
        HAL_UART_Receive_DMA(huart,dma_vision_buf,DMA_VISION_LEN);
    }
}

/**
  * @brief ���ڿ����ж�   ע������it.c��ÿ�����ڵ��ж��е��øú���
  * @param UART_HandleTypeDef *huart
  * @retval ��
  */
void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) //�ж��Ƿ��ǿ����ж�
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart);                     //��������жϱ�־�������һֱ���Ͻ����жϣ�
			HAL_UART_AbortReceive(huart);						//���رս����ж�
//        HAL_UART_DMAStop(huart);															//ֹͣ����DMA����
        USER_UART_IDLECallback(huart);                     //���ô��ڹ��ܻص�����
    }
}


/**
* @brief  ���ڳ�ʼ��:ʹ�ܴ��ڿ����ж�,��������DMA����
* @param  ��
* @retval ��
*/
void USER_UART_Init()
{
    __HAL_UART_CLEAR_IDLEFLAG(&SBUS_HUART);
    __HAL_UART_ENABLE_IT(&SBUS_HUART, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&SBUS_HUART, dma_sbus_buf, DMA_SBUS_LEN);
    
	  __HAL_UART_CLEAR_IDLEFLAG(&NEW_RC_HUART);
    __HAL_UART_ENABLE_IT(&NEW_RC_HUART, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&NEW_RC_HUART, dma_new_rc_buf, VTM_DATA_LEN);//ͼ����·

    /* USART6的RX方向DMA(hdma_usart6_rx)从未配置，IDLE回调走到USART6分支会野指针解引用触发HardFault。
       视觉模块还没接，先不开IDLE中断/不收数据，只保留VOFA的TX输出。等补上RX DMA配置后再恢复 */
//    __HAL_UART_CLEAR_IDLEFLAG(&VISION_HUART);
//    __HAL_UART_ENABLE_IT(&VISION_HUART, UART_IT_IDLE);
//    HAL_UART_Receive_DMA(&VISION_HUART,dma_vision_buf,DMA_VISION_LEN);

    /* 8路灰度传感器已改用纯GPIO读取，USART10不再初始化，这里不能再调用 */

	log_init(&huart6);  /* VOFA改用U6 */
}

