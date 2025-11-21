/**
  ******************************************************************************
  * @file    usart_comm.c
  * @author  ZZY 1950828677@qq.com
  * @brief   
  * @date    2025-10
  * @version 1.0
  * @
  ==============================================================================
*/

/* Includes ------------------------------------------------------------------*/
#include "usart_comm.h"

/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
//uint8_t Uart1_Rx_Buff[USART1_RX_BUFFER_SIZE];
//uint8_t Uart2_Rx_Buff[USART2_RX_BUFFER_SIZE];     /*!< Receive buffer for Uart2 */
//uint8_t Uart3_Rx_Buff[USART3_RX_BUFFER_SIZE];     /*!< Receive buffer for Uart3 */
//uint8_t Uart4_Rx_Buff[USART4_RX_BUFFER_SIZE];     /*!< Receive buffer for Uart4 */
//uint8_t Uart10_Rx_Buff[USART5_RX_BUFFER_SIZE];     /*!< Receive buffer for Uart5 */
//uint8_t Uart6_Rx_Buff[USART6_RX_BUFFER_SIZE];     /*!< Receive buffer for Uart6 */

volatile uint8_t Uart1_Rx_Buff[USART1_RX_BUFFER_SIZE] __attribute__((section(".RAM_D1")));
volatile uint8_t Uart2_Rx_Buff[USART2_RX_BUFFER_SIZE] __attribute__((section(".RAM_D1")));
volatile uint8_t Uart3_Rx_Buff[USART3_RX_BUFFER_SIZE] __attribute__((section(".RAM_D1")));
volatile uint8_t Uart4_Rx_Buff[USART4_RX_BUFFER_SIZE] __attribute__((section(".RAM_D1")));
volatile uint8_t Uart6_Rx_Buff[USART6_RX_BUFFER_SIZE] __attribute__((section(".RAM_D1")));
volatile uint8_t Uart10_Rx_Buff[USART10_RX_BUFFER_SIZE] __attribute__((section(".RAM_D1")));
/* 强制把数组首字节改成一个魔法数 */



uint32_t i;
volatile  uint32_t j;
/* Private type --------------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/
static uint32_t User_USART1_RxCpltCallback(uint8_t *buf, uint16_t len);
static uint32_t User_USART6_RxCpltCallback(uint8_t *buf, uint16_t len);

/* function prototypes -------------------------------------------------------*/


void comm_usart_init(void)
{
    bsp_user_usart_Init(&huart1,Uart1_Rx_Buff,USART1_RX_BUFFER_SIZE,User_USART1_RxCpltCallback);

    // bsp_user_usart_Init(&huart1,Uart1_Rx_Buff,USART1_RX_BUFFER_SIZE,rc_callback_handler);
    // bsp_user_usart_Init(&huart1,Uart1_Rx_Buff,USART1_RX_BUFFER_SIZE,rc_callback_handler);
//    bsp_user_usart_Init(&huart6,Uart6_Rx_Buff,USART6_RX_BUFFER_SIZE,User_USART6_RxCpltCallback);
Uart1_Rx_Buff[0] = 0xAB;

}

/*-----------------------------USART-----------------------------------*/
static uint32_t User_USART1_RxCpltCallback(uint8_t *buf, uint16_t len)
{
  rc_callback_handler(&rc,buf);   

  memset(buf,0,sizeof(buf));  


	return 0;
}

static uint32_t User_USART6_RxCpltCallback(uint8_t *buf, uint16_t len)
{
//  new_rc_callback_handler();   
//  memset(dr16_dma_buf,0,sizeof(dr16_dma_buf));  
	return 0;
}

//void User_USART6_RxCpltCallback(uint8_t *buf, uint16_t len)
//{
//   //????????????
//}

//void User_USART10_RxCpltCallback(uint8_t *buf, uint16_t len)
//{
//   //????????????
//}
