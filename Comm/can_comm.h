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

#ifndef __CAN_COMM_H
#define __CAN_COMM_H


#ifdef __cplusplus
extern "C" {
#endif


#ifdef  __BSP_CAN_GLOBALS
#define __BSP_CAN_EXT
#else
#define __BSP_CAN_EXT extern
#endif

/* Includes ------------------------------------------------------------------*/
#include "bsp_can.h"
#include "bsp_usart.h"


#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include "prot_rm_motor.h"
#include "prot_dm_motor.h"
#include "prot_T_imu.h"

#include "can_message.h"

/* Private macros ------------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/


typedef enum
{
    CAN_3508_FRIC_L_ID       = 0x201,
    CAN_3508_FRIC_R_ID       = 0x202,
    CAN_TRIGGER_MOTOR1_ID = 0x203,
	
    CAN_YAW_MOTOR_ID      = 0x206,//2025��1��17�� �۲췢��Ŀǰ
    CAN_J4310_MOTOR_ID      = 0x016,
	
	CAN_J4310_ID=0X06,//J4310�������֡ID   
	CAN_MG4005_T_ID=0x147 ,//2025��1��6�գ�Ԥ����pit���MG4005��idΪ0x07�����������뿪�ص���ֵ �༴motorId��   �� 0x140(���ҹ涨) + 0x07 
	CAN_MG4005_R_ID=0x187,//2025��1��10�� �����ҹ涨��CAN����id��0x180+id���ٵģ���ʵ����0x140  

} can_msg_id_e;

/* Exported function declarations --------------------------------------------*/
void comm_can_init(void);
void User_CAN1_RxCpltCallback(CAN_RxBuffer_t* buf);
void User_CAN2_RxCpltCallback(CAN_RxBuffer_t* buf);
void User_CAN3_RxCpltCallback(CAN_RxBuffer_t* buf);
void Prot_CAN1_Send_Frame(uint32_t id, const uint8_t *data, uint8_t dlc);
void Prot_CAN2_Send_Frame(uint32_t id, const uint8_t *data, uint8_t dlc);
void Prot_CAN3_Send_Frame(uint32_t id, const uint8_t *data, uint8_t dlc);
void Task_CAN1Transmit(void const *arg);
void Task_CAN2Transmit(void const *arg);
void Task_CAN3Transmit(void const *arg);

extern uint32_t trigger_can_receive_cnt;
extern uint32_t fric_can_recive_cnt;
extern uint32_t yaw_can_receive_cnt;

#endif
