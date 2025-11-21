/**
  ******************************************************************************
  * @file   
  * @author 
  * @brief  
  ******************************************************************************
  * @attention
  *****************
*/

#ifndef __BSP_USART_H
#define __BSP_USART_H


#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/

#if defined(STM32F405xx) || defined(STM32F407xx)
	#include "stm32f4xx_hal.h"
#endif
#if defined(STM32F103xx)
	#include "stm32f1xx_hal.h"
#endif
#if defined(STM32H723xx)
#include "stm32h7xx.h"
#endif	

#include "usart.h"



/* Private macros ------------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
typedef uint32_t (*usart_call_back)(uint8_t *buf, uint16_t len);

/** 
* @brief 
*/
typedef struct
{
  UART_HandleTypeDef *husart;
  uint16_t rx_buffer_size;
  uint8_t *rx_buffer;
  usart_call_back callback_f;
}usart_user_rx_handle_t;

/**
* @brief 
*/
//typedef struct
//{
//  uint8_t port_num;
//  uint16_t len;
//  void* address;
//} USART_COB;


/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern usart_user_rx_handle_t usart1_rx_user_handle;
extern usart_user_rx_handle_t usart2_rx_user_handle;
extern usart_user_rx_handle_t usart3_rx_user_handle;
extern usart_user_rx_handle_t usart9_rx_user_handle;
extern usart_user_rx_handle_t usart6_rx_user_handle;
extern usart_user_rx_handle_t usart10_rx_user_handle;

/* Exported function declarations --------------------------------------------*/
void bsp_user_usart_Init(UART_HandleTypeDef *huart, uint8_t *Rxbuffer, uint32_t length, usart_call_back fun);
void bsp_usart_rx_callback_register( usart_user_rx_handle_t  *p,usart_call_back callback);
void bsp_user_usart_Receive_Handler(usart_user_rx_handle_t  *p);
void Uart_Receive_Handler(usart_user_rx_handle_t  *p);
uint32_t Uart1_Transmit(uint8_t *msg, uint16_t len);
uint32_t Uart2_Transmit(uint8_t *msg, uint16_t len);
uint32_t Uart3_Transmit(uint8_t *msg, uint16_t len);
uint32_t Uart9_Transmit(uint8_t *msg, uint16_t len);
uint32_t Uart10_Transmit(uint8_t *msg, uint16_t len);
uint32_t Uart6_Transmit(uint8_t *msg, uint16_t len);


#endif
