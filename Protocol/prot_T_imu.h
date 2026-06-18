#ifndef __BSP_T_IMU_H
#define __BSP_T_IMU_H

#include "bsp_can.h"
#include "gimbal_task.h"
#include "string.h"
#include "cmsis_os.h"
#include "task.h"
#include "main.h"

#define OLD_IMU 0
#define TIMU_PALSTANCE_ID   0x017//
#define TIMU_ANGLE_ID   	  0x018//0x002
#define TIMU_ROL_ID   	  0x019//0x002

extern int sys_us[2];
extern int incement;
extern int err[2];

typedef struct
{
    float pitch;
    float yaw;
	  float roll;
	float last_yaw;
	float real_yaw;
	float cnt;
    float wy;
    float wz;
	float wx;
    float az;
    float kal_az;
	  float msg_cnt;
	  float yaw_temp;
} Taurus_imu_data_t;

extern uint32_t imucnt;
extern Taurus_imu_data_t   imu_data;
void T_imu_calcu(uint32_t can_id,uint8_t * CAN_Rx_data);

#endif

