/**
  * @file arm_yaw_limit.c
  * @brief 机械臂 yaw 编码器电子限位
  * @note 参考云台的多圈编码器限位：反馈相对上电原点累计，边界越界只允许向回运动。
  *       配置未写入前不阻止手动采集；输出层安全由 arm_task 强制零电流。
  */
#include "arm_yaw_limit.h"
#include <string.h>

arm_yaw_limit_config_t arm_yaw_limit_config;
arm_yaw_feedback_t arm_yaw_feedback;
static uint8_t s_origin_valid;

static void update_feedback(void)
{
    arm_yaw_feedback.ecd = arm_yaw_motor.ecd;
    arm_yaw_feedback.total_count = arm_yaw_motor.total_ecd;
    arm_yaw_feedback.round_count = arm_yaw_motor.round_cnt;
    arm_yaw_feedback.angle_deg = (float)arm_yaw_feedback.total_count * ARM_YAW_DEG_PER_COUNT;
    arm_yaw_feedback.speed_rpm = arm_yaw_motor.speed_rpm;
    arm_yaw_feedback.current_raw = arm_yaw_motor.rx_current;
    arm_yaw_feedback.temperature = arm_yaw_motor.temperature;
    arm_yaw_feedback.receive_count = arm_yaw_motor.receive_cnt;
    arm_yaw_feedback.online = arm_yaw_motor.online;
}

void arm_yaw_limit_init(void)
{
    memset(&arm_yaw_limit_config, 0, sizeof(arm_yaw_limit_config));
    memset(&arm_yaw_feedback, 0, sizeof(arm_yaw_feedback));
    s_origin_valid = 0U;
    (void)arm_yaw_limit_set(ARM_YAW_ECD_LEFT_DEFAULT, ARM_YAW_ECD_RIGHT_DEFAULT);
}

int32_t arm_yaw_limit_clamp_target(int32_t target_ecd)
{
    update_feedback();
    if (!arm_yaw_limit_config.configured) {
        return target_ecd;
    }
    if (target_ecd > arm_yaw_limit_config.upper_ecd) {
        return arm_yaw_limit_config.upper_ecd;
    }
    if (target_ecd < arm_yaw_limit_config.lower_ecd) {
        return arm_yaw_limit_config.lower_ecd;
    }
    return target_ecd;
}

uint8_t arm_yaw_limit_is_safe(int32_t output_current)
{
    update_feedback();
    if (!s_origin_valid) {
        s_origin_valid = 1U;
    }
    if (!arm_yaw_feedback.online) {
        return 0U;
    }
    if (!arm_yaw_limit_config.configured) {
        return 1U;
    }
    int32_t current = arm_yaw_feedback.ecd;
    if (current >= arm_yaw_limit_config.upper_ecd && output_current > 0) {
        return 0U;
    }
    if (current <= arm_yaw_limit_config.lower_ecd && output_current < 0) {
        return 0U;
    }
    return 1U;
}

int8_t arm_yaw_limit_side(void)
{
    update_feedback();
    if (!arm_yaw_limit_config.configured) {
        return 0;
    }
    if (arm_yaw_feedback.ecd >= arm_yaw_limit_config.upper_ecd) {
        return 1;
    }
    if (arm_yaw_feedback.ecd <= arm_yaw_limit_config.lower_ecd) {
        return -1;
    }
    return 0;
}

int arm_yaw_limit_set(int32_t lower_ecd, int32_t upper_ecd)
{
    if (lower_ecd < 0 || upper_ecd > ARM_YAW_COUNTS_PER_REV ||
        lower_ecd >= upper_ecd) {
        return -1;
    }
    arm_yaw_limit_config.lower_ecd = lower_ecd;
    arm_yaw_limit_config.upper_ecd = upper_ecd;
    arm_yaw_limit_config.configured = 1U;
    return 0;
}

void arm_yaw_limit_clear(void)
{
    arm_yaw_limit_config.configured = 0U;
}
