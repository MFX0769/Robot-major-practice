/**
  ******************************************************************************
  * @file    bsp_imu.h
  * @author  taurus
  * @brief   BMI088 IMU BSP: SPI2 + DMA, DRDY-based 1kHz sampling, Mahony AHRS,
  *          thermostat control, unified data interface via FreeRTOS queue.
  * @attention
  *   Ported from vendor CtrBoard-H7_ALL (2026-11-22). Pins (ioc authority):
  *     SPI2 SCK=PB13 / MISO=PC2_C / MOSI=PC1, ACC_CS=PC0, GYRO_CS=PC3
  *     ACC_INT=PE10, GYRO_INT=PE12 (DRDY, currently polled via status reg)
  ******************************************************************************
*/
#ifndef __BSP_IMU_H
#define __BSP_IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/*============================== 配置 ==============================*/
#define IMU_TARGET_RATE_HZ      1000U   /* 目标结算频率 1kHz */
#define IMU_MAHONY_UPDATE_HZ    1000U   /* Mahony 融合频率 */
#define IMU_TEMP_TARGET_C       40.0f   /* 恒温目标 40°C (原厂一致) */
#define IMU_QUEUE_LENGTH        4       /* IMU 数据队列深度 */

/*============================== 数据类型 ==============================*/
/* 单次 IMU 采样数据（统一接口，供 queue 传递 / 上层消费） */
typedef struct {
    float gyro[3];       /* 角速度 rad/s, x/y/z */
    float accel[3];      /* 加速度 m/s^2, x/y/z */
    float angle[3];      /* 欧拉角 rad: [yaw, pitch, roll] */
    float quat[4];       /* 四元数 w/x/y/z */
    float temp;          /* IMU 温度 °C */
    uint32_t timestamp_us; /* 采样时间戳 (微秒, 基于 DWT CYCCNT) */
    uint32_t seq;        /* 采样序号, 单调递增, 用于新旧判据 */
    uint8_t  fresh;      /* 1=本次读到新采样(DRDY), 0=复用上次旧值 */
} imu_data_t;

/* BSP 初始化/运行状态 */
typedef enum {
    IMU_OK = 0,
    IMU_ERR_INIT = 1,
    IMU_ERR_NO_SENSOR = 2,
    IMU_ERR_SPI = 3,
    IMU_ERR_TIMEOUT = 4,
} imu_status_t;

typedef struct {
    imu_status_t status;
    uint32_t init_error_code;   /* BMI088_init() 返回值, 0=OK */
    uint32_t sample_cnt;        /* 累计采样数 */
    uint32_t fresh_cnt;         /* 新采样次数 */
    uint32_t stale_cnt;         /* 复用旧值次数 */
    uint32_t queue_overflow;    /* 队列满次数 */
    float    actual_rate_hz;    /* 实测结算频率 */
} imu_diag_t;

/*============================== 对外 API ==============================*/
/* 初始化: SPI2+DMA+BMI088+Mahony+温控PWM, 创建数据队列; 返回状态 */
imu_status_t bsp_imu_init(void);

/* IMU 任务主循环 (FreeRTOS task entry), 周期由 IMU_TASK_PERIOD_MS 决定 */
void bsp_imu_task(void const *argument);

/* 获取最新一份 IMU 数据 (拷贝, 线程安全); 返回 0=成功 */
int  bsp_imu_get_latest(imu_data_t *out);

/* 从队列取一份数据 (阻塞/超时); pdTRUE=成功. 供需要逐帧消费的模块 */
int  bsp_imu_receive(imu_data_t *out, uint32_t timeout_ms);

/* 获取诊断信息 */
const imu_diag_t *bsp_imu_get_diag(void);

/* 供调试: 直接读原始 gyro/acc/temp (不经 Mahony), 单位 rad/s, m/s^2, °C */
void bsp_imu_read_raw(float gyro[3], float accel[3], float *temp);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_IMU_H */
