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
    /* 底盘2006电机ID */
    CAN_CHASSIS_LF_ID     = 0x204,  // 左前
    CAN_CHASSIS_LB_ID     = 0x202,  // 左后
    CAN_CHASSIS_RF_ID     = 0x203,  // 右前
    CAN_CHASSIS_RB_ID     = 0x201,  // 右后
    
    /* Yaw 6020电机ID */
    CAN_YAW_MOTOR_ID      = 0x206,
    
    /* 机械臂底座 6020 电机ID */
    CAN_ARM_YAW_MOTOR_ID  = 0x208,

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
void chassis_can_send(void);

extern uint32_t trigger_can_receive_cnt;
extern uint32_t fric_can_recive_cnt;
extern uint32_t yaw_can_receive_cnt;

#endif
