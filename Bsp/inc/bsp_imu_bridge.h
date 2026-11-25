/**
  ******************************************************************************
  * @file    bsp_imu_bridge.h
  * @brief   IMU data bridge: vendor imu_task (BMI088, rad) -> taurus imu_data (deg)
  * @note    2026-11-22 migration. Vendor App/imu_task.c publishes imuAngle[] (rad)
  *          and gyro[] (rad/s) at 1kHz; business tasks consume Taurus_imu_data_t.
  ******************************************************************************
*/
#ifndef __BSP_IMU_BRIDGE_H
#define __BSP_IMU_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

/* 在原厂 ImuTask 循环里每 1ms 调用一次, 把姿态/角速度换算进业务 imu_data */
void bsp_imu_bridge_update(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_IMU_BRIDGE_H */
