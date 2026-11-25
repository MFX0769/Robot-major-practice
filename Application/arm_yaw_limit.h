/**
  * @file arm_yaw_limit.h
  * @brief 机械臂 yaw 编码器电子限位与采集接口
  * @note 仅使用 GM6020 反馈，无 IMU；上电原点、左右边界和电流均在 RAM。
  */
#ifndef __ARM_YAW_LIMIT_H__
#define __ARM_YAW_LIMIT_H__

#include <stdint.h>
#include "prot_rm_motor.h"

/* 6020 机械角 0~8191 对应一圈，8192=360°。 */
#define ARM_YAW_COUNTS_PER_REV  8192L
#define ARM_YAW_DEG_PER_COUNT   (360.0f / ARM_YAW_COUNTS_PER_REV)

/* 2026-11-25 手动扫极限实测：右 ECD 6478、左 ECD 1260，区间不跨 8191→0。 */
#define ARM_YAW_ECD_LEFT_DEFAULT   1260
#define ARM_YAW_ECD_RIGHT_DEFAULT  6478

/* 采集周期 50ms（20Hz）。 */
#define ARM_YAW_LIMIT_LOG_PERIOD_MS  50U

typedef struct {
    int32_t lower_ecd;         /* 下边界原始 ECD */
    int32_t upper_ecd;         /* 上边界原始 ECD */
    uint8_t configured;        /* 左右边界已配置 */
} arm_yaw_limit_config_t;

typedef struct {
    int32_t ecd;               /* 原始单圈编码器 */
    int32_t total_count;       /* 相对上电原点的累计编码器 */
    int32_t round_count;       /* 跨零圈数 */
    float angle_deg;           /* 相对上电原点的累计角度 */
    int16_t speed_rpm;
    int16_t current_raw;
    uint8_t temperature;
    uint32_t receive_count;
    uint8_t online;
} arm_yaw_feedback_t;

extern arm_yaw_limit_config_t arm_yaw_limit_config;
extern arm_yaw_feedback_t arm_yaw_feedback;

/** @brief 初始化限位状态，默认未配置 / 初始化边界 */
void arm_yaw_limit_init(void);

/** @brief 读取当前反馈快照，按边界夹取目标 / 读取当前反馈快照 */
int32_t arm_yaw_limit_clamp_target(int32_t target_count);

/** @brief 判断反馈是否新鲜，输出是否应归零 / 判断反馈是否新鲜 */
uint8_t arm_yaw_limit_is_safe(int32_t output_current);

/** @brief 返回边界状态：-1 下界，1 上界，0 边界内 / 返回边界状态 */
int8_t arm_yaw_limit_side(void);

/** @brief 设置左右边界并校验顺序 / 设置左右边界 */
int arm_yaw_limit_set(int32_t lower_count, int32_t upper_count);

/** @brief 清除边界，恢复仅采集模式 / 清除边界 */
void arm_yaw_limit_clear(void);

#endif /* __ARM_YAW_LIMIT_H__ */
