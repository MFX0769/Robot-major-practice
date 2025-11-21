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


/* Private macros ------------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
//typedef struct
//{
//  uint16_t ecd;
//  uint16_t last_ecd;

//  int16_t speed_rpm;
//  int16_t given_current;

//  int32_t round_cnt;
//  int32_t total_ecd;

//  uint16_t offset_ecd;
//  uint32_t msg_cnt;
//} moto_measure_t;

//__BSP_CAN_EXT moto_measure_t moto_fric[2] ;
//__BSP_CAN_EXT moto_measure_t moto_yaw;
//__BSP_CAN_EXT moto_measure_t motor_trigger;

typedef enum
{
    CAN_3508_FRIC_L_ID       = 0x201,
    CAN_3508_FRIC_R_ID       = 0x202,
    CAN_TRIGGER_MOTOR1_ID = 0x207,
	
    CAN_YAW_MOTOR_ID      = 0x206,//2025年1月17日 观察发现目前
    CAN_J4310_MOTOR_ID      = 0x016,
	
	CAN_J4310_ID=0X06,//J4310电机控制帧ID   
	CAN_MG4005_T_ID=0x147 ,//2025年1月6日：预计新pit电机MG4005的id为0x07（电机背面编码开关的数值 亦即motorId）   即 0x140(厂家规定) + 0x07 
	CAN_MG4005_R_ID=0x187,//2025年1月10日 ：厂家规定的CAN反馈id是0x180+id，假的，其实还是0x140  

} can_msg_id_e;

/* Exported function declarations --------------------------------------------*/
void comm_can_init(void);
void User_CAN1_RxCpltCallback(CAN_RxBuffer_t* buf);
void User_CAN2_RxCpltCallback(CAN_RxBuffer_t* buf);
void User_CAN3_RxCpltCallback(CAN_RxBuffer_t* buf);
void Task_CAN1Transmit(void *arg);
void Task_CAN2Transmit(void *arg);
void Task_CAN3Transmit(void *arg);

//void encoder_data_handler(moto_measure_t *ptr, FDCAN_HandleTypeDef *hfdcan, uint8_t *CAN_Rx_data);
#endif
