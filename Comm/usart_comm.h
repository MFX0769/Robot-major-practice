/**
  ******************************************************************************
  * @file    bsp.h
  * @author  ZZY 1950828677@qq.com
  * @brief   
  * @date    2025-10
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
#include "bsp_usart.h"

#include "prot_dr16.h"
#include "prot_new_rc.h"
#include "prot_rm_judge.h"
#include "prot_vision.h"
/* Private macros ------------------------------------------------------------*/
#define USART1_RX_BUFFER_SIZE 18  //dt7遥控数据长度
#define USART2_RX_BUFFER_SIZE 100
#define USART3_RX_BUFFER_SIZE 21  //新遥控数据长度
#define USART9_RX_BUFFER_SIZE 256
#define USART10_RX_BUFFER_SIZE 28   //vision数据长度
#define USART6_RX_BUFFER_SIZE 18
/* Private type --------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/


//extern uint8_t Uart1_Rx_Buff[USART1_RX_BUFFER_SIZE];
//extern uint8_t Uart1_Rx_Buff[USART1_RX_BUFFER_SIZE];
//extern uint8_t Uart2_Rx_Buff[USART2_RX_BUFFER_SIZE];
//extern uint8_t Uart3_Rx_Buff[USART3_RX_BUFFER_SIZE];
//extern uint8_t Uart4_Rx_Buff[USART4_RX_BUFFER_SIZE];
//extern uint8_t Uart5_Rx_Buff[USART5_RX_BUFFER_SIZE];
//extern uint8_t Uart6_Rx_Buff[USART6_RX_BUFFER_SIZE];

extern volatile uint8_t Uart1_Rx_Buff[USART1_RX_BUFFER_SIZE] __attribute__((section(".RAM_D1")));
extern volatile uint8_t Uart2_Rx_Buff[USART2_RX_BUFFER_SIZE] __attribute__((section(".RAM_D1")));
extern volatile uint8_t Uart3_Rx_Buff[USART3_RX_BUFFER_SIZE] __attribute__((section(".RAM_D1")));
extern volatile uint8_t Uart9_Rx_Buff[USART9_RX_BUFFER_SIZE] __attribute__((section(".RAM_D1")));
extern volatile uint8_t Uart6_Rx_Buff[USART6_RX_BUFFER_SIZE] __attribute__((section(".RAM_D1")));
extern volatile uint8_t Uart10_Rx_Buff[USART10_RX_BUFFER_SIZE] __attribute__((section(".RAM_D1")));


/* Exported function declarations --------------------------------------------*/
void comm_usart_init(void);

#endif
