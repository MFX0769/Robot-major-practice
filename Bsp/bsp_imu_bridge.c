/**
  ******************************************************************************
  * @file    bsp_imu_bridge.c
  * @brief   IMU data bridge implementation (2026-11-22)
  ******************************************************************************
*/
#include "bsp_imu_bridge.h"
#include "prot_T_imu.h"

/* 原厂 imu_task.c 的全局输出 (App/imu_task.c) */
extern float imuAngle[3];   /* [yaw, pitch, roll] rad */
extern float gyro[3];       /* rad/s */

#define RAD2DEG 57.29578f

void bsp_imu_bridge_update(void)
{
    /* 单位换算 rad -> deg; 坐标系校准现场按实物标定 */
    imu_data.pitch = imuAngle[1] * RAD2DEG;
    imu_data.yaw   = imuAngle[0] * RAD2DEG;
    imu_data.roll  = imuAngle[2] * RAD2DEG;
    imu_data.wx    = gyro[0] * RAD2DEG;
    imu_data.wy    = gyro[1] * RAD2DEG;
    imu_data.wz    = gyro[2] * RAD2DEG;
    imucnt++;
}
