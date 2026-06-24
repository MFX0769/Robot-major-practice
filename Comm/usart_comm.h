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
#include "prot_vtm.h"
#include "prot_rm_judge.h"
#include "prot_vision.h"
#include "data_log.h"
#include "log_task.h"
/* Private macros ------------------------------------------------------------*/
//#define USART1_RX_BUFFER_SIZE 18  //dt7遥控数据长度
//#define USART2_RX_BUFFER_SIZE 100
//#define USART3_RX_BUFFER_SIZE 21  //新遥控数据长度
//#define USART9_RX_BUFFER_SIZE 256
//#define USART10_RX_BUFFER_SIZE 28   //vision数据长度
//#define USART6_RX_BUFFER_SIZE 18

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

//extern volatile uint8_t Uart1_Rx_Buff[USART1_RX_BUFFER_SIZE] __attribute__((section(".RAM_D1")));
//extern volatile uint8_t Uart2_Rx_Buff[USART2_RX_BUFFER_SIZE] __attribute__((section(".RAM_D1")));
//extern volatile uint8_t Uart3_Rx_Buff[USART3_RX_BUFFER_SIZE] __attribute__((section(".RAM_D1")));
//extern volatile uint8_t Uart9_Rx_Buff[USART9_RX_BUFFER_SIZE] __attribute__((section(".RAM_D1")));
//extern volatile uint8_t Uart6_Rx_Buff[USART6_RX_BUFFER_SIZE] __attribute__((section(".RAM_D1")));
//extern volatile uint8_t Uart10_Rx_Buff[USART10_RX_BUFFER_SIZE] __attribute__((section(".RAM_D1")));


/* Exported function declarations --------------------------------------------*/
void comm_usart_init(void);


/*---------------------------------------------分界线------------------------*/
#define 	DBUS_HUART    huart1   //真正DBUS协议用(若以后切回/换硬件)
#define 	SBUS_HUART    huart1   //当前USART1实际接的是SBUS接收机，跟DBUS_HUART是同一个硬件口
#define   NEW_RC_HUART  huart9
#define 	TUCHUAN_HUART		huart2		//8路灰度传感器
#define DEBUG_HUART	huart6 
#define LOG_HUART huart3
#define   VISION_HUART	huart10

#define  DMA_DBUS_LEN			18     //保留给真正的DBUS协议(rc_callback_handler)，跟D:\侧一致
#define  DMA_SBUS_LEN			36     //SBUS一帧25字节(SBUS_FRAME_SIZE)，留余量。现在USART1实际跑SBUS，要用这个，不是DMA_DBUS_LEN
#define VTM_DATA_LEN    21     //放在遥控器解算的头文件里
#define DMA_8IR_LEN      32     //8路灰度传感器
#define  DMA_VISION_LEN		40     //原23  2025年4月1
void USER_UART_Init(void);
void USER_UART_IRQHandler(UART_HandleTypeDef *huart);
void USER_UART_IDLECallback(UART_HandleTypeDef *huart);


#endif
