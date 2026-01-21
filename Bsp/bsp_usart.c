/**
  ******************************************************************************
  * @file    bsp_usart.c
  * @author  ZZY 1950828677@qq.com
  * @brief   Code for USART driver in STM32 series MCU
  * @date    2025-09
  * @version 1.0
  * @
  ==============================================================================
*/

/* Includes ------------------------------------------------------------------*/
#include "bsp_usart.h"

/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* ʵ�����ṹ�� ��ʼ�����ֳ�Ա---------------------------------------------------------*/
usart_user_rx_handle_t usart1_rx_user_handle =    //DBUS
{
		.callback_f = NULL};

usart_user_rx_handle_t usart2_rx_user_handle =    //JUDGE
{
		.callback_f = NULL};

usart_user_rx_handle_t usart3_rx_user_handle =    //AIR UNIT ͼ��
{
  .callback_f = NULL};

usart_user_rx_handle_t usart9_rx_user_handle =    //VISION(backup)
{
  .callback_f = NULL};

usart_user_rx_handle_t usart10_rx_user_handle =    //SD_LOG
{
    .callback_f = NULL};

usart_user_rx_handle_t usart6_rx_user_handle =    //DEBUG
{
    .callback_f = NULL};
/* Private type --------------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/
static void bsp_user_usart_Rx_Idle_Callback(usart_user_rx_handle_t  *p);
// static void Error_Handler(void);

/* function prototypes -------------------------------------------------------*/
/**
* @brief  Initialize uart device
* @param  *huart: pointer of uart IRQHandler
* @param	Rxbuffer: user buffer array
* @param	length: the length of array
* @retval None
*/
void bsp_user_usart_Init(UART_HandleTypeDef *huart, uint8_t *Rxbuffer, uint32_t length, usart_call_back fun)
{
	if (huart == NULL)
		Error_Handler();
	else {}

	if(huart->Instance == USART1 )
	{
		usart1_rx_user_handle.rx_buffer = Rxbuffer;
		usart1_rx_user_handle.rx_buffer_size = length;
		usart1_rx_user_handle.husart = huart;
    usart1_rx_user_handle.callback_f = fun;
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
		HAL_UART_Receive_DMA(huart, Rxbuffer, length);
	}

	else if (huart->Instance == USART2)
	{
		usart2_rx_user_handle.rx_buffer = Rxbuffer;
		usart2_rx_user_handle.rx_buffer_size = length;
		usart2_rx_user_handle.husart = huart;
    usart2_rx_user_handle.callback_f = fun;
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
		HAL_UART_Receive_DMA(huart, Rxbuffer, length);
	}
    
	else if (huart->Instance == USART3 )
	{
		usart3_rx_user_handle.rx_buffer = Rxbuffer;
		usart3_rx_user_handle.rx_buffer_size = length;
		usart3_rx_user_handle.husart = huart;
    usart3_rx_user_handle.callback_f = fun;
		__HAL_UART_ENABLE_IT(huart,UART_IT_IDLE);
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		HAL_UART_Receive_DMA(huart, Rxbuffer, length);
	}
    
	else if (huart->Instance == UART4)
	{
		usart9_rx_user_handle.rx_buffer = Rxbuffer;
		usart9_rx_user_handle.rx_buffer_size = length;
		usart9_rx_user_handle.husart = huart;
    usart9_rx_user_handle.callback_f = fun;
		__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		HAL_UART_Receive_DMA(huart, Rxbuffer, length);
	}
   
	else if (huart->Instance == USART10)
	{
		usart10_rx_user_handle.rx_buffer = Rxbuffer;
		usart10_rx_user_handle.rx_buffer_size = length;
		usart10_rx_user_handle.husart = huart;
    usart10_rx_user_handle.callback_f = fun;
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
		HAL_UART_Receive_DMA(huart, Rxbuffer, length);
	}
   
	else if (huart->Instance == USART6)  //u6��debug���ݣ�������յ�����
	{
		// usart6_rx_user_handle.rx_buffer = Rxbuffer;
		// usart6_rx_user_handle.rx_buffer_size = length;
		// usart6_rx_user_handle.uart_h = huart;
    // usart6_rx_user_handle.call_back_f = fun;
		// __HAL_UART_CLEAR_IDLEFLAG(huart);
		// __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
		// HAL_UART_Receive_DMA(huart, Rxbuffer, length);
	}
	
	else
		// Error_Handler();
    return;
}


/**
 * @brief  Registered user callback function  ���ڻص�����ע��
 * @param  p:        serial port handle
 * @param  callback: user callback function
 * @retval None
 */
void bsp_usart_rx_callback_register(  usart_user_rx_handle_t  *p,
                                      usart_call_back         callback)
{
    /* Check the parameters */
	assert_param(fun != NULL);
	assert_param(m_obj != NULL);
    p->callback_f= callback;
}


/**
 * @brief   �û����ڷ���ISR
 * @param   p: serial port handle
 * @retval  None
 */
void bsp_user_usart_Receive_Handler(usart_user_rx_handle_t  *p)
{
	if(__HAL_UART_GET_FLAG(p->husart,UART_FLAG_IDLE)!=RESET)
	{
		HAL_UART_AbortReceive(p->husart);
		bsp_user_usart_Rx_Idle_Callback(p);
	}
}

/**
 * @brief   �û������жϴ���&��������
 * @note    call in uart_receive_handler() function
 * @param   uart IRQHandler id
 * @retval  None
 */
static void bsp_user_usart_Rx_Idle_Callback(usart_user_rx_handle_t  *p)
{
  /* Check the parameters */
	// assert_param(m_obj != NULL);
	
  /* Private variables */
	static uint16_t usart_rx_num;

  /* clear idle it flag avoid idle interrupt all the time */
	__HAL_UART_CLEAR_IDLEFLAG(p->husart);

  /* clear DMA transfer complete flag */
	// HAL_UART_DMAStop(p->husart);  //�ر�ȫ��DMA�Ļ����ܻ�Ӱ�췢��   �Թ���DMAStop�Ļ���С���Զ˵Ľ���ÿ������ͻ���һ֡
  HAL_UART_AbortReceive(p->husart);


  /* handle received data in user decode ISR */
	usart_rx_num = p->rx_buffer_size - ((DMA_Stream_TypeDef*)p->husart->hdmarx->Instance)->NDTR;
	if(p->callback_f != NULL)
		p->callback_f(p->rx_buffer, usart_rx_num);
	
	HAL_UART_Receive_DMA(p->husart, p->rx_buffer, p->rx_buffer_size);
  

}


/**
 * @brief Transmit function for specific Uart.
 * @param msg Message content to send.
 * @param len Message len to send.
 * @retval HAL Status
 */
uint32_t Uart1_Transmit(uint8_t *msg, uint16_t len)
{
  return HAL_UART_Transmit_DMA(&huart1, msg, len);
}
uint32_t Uart2_Transmit(uint8_t *msg, uint16_t len)
{
  return HAL_UART_Transmit_DMA(&huart2, msg, len);
}
uint32_t Uart3_Transmit(uint8_t *msg, uint16_t len)
{
  return HAL_UART_Transmit_DMA(&huart3, msg, len);
}
uint32_t Uart9_Transmit(uint8_t *msg, uint16_t len)
{
  return HAL_UART_Transmit_DMA(&huart9, msg, len);
}
uint32_t Uart10_Transmit(uint8_t *msg, uint16_t len)
{
  return HAL_UART_Transmit_DMA(&huart10, msg, len);
}
uint32_t Uart6_Transmit(uint8_t *msg, uint16_t len)
{
  return HAL_UART_Transmit_DMA(&huart6, msg, len);
}

