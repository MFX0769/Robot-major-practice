/**
  * @file    arm_ik.c
  * @brief   机械臂逆运动学实现
  * @note    4轴几何闭式解，O(1)复杂度
  *          底座yaw + 肩关节 + 肘关节 + 夹爪
  */

#include "arm_ik.h"
#include "arm_task.h"
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

/* 臂长参数 (可在Keil Debug中修改) */
static float L1 = ARM_L1;
static float L2 = ARM_L2;

/**
  * @brief  逆运动学初始化
  */
void arm_ik_init(void)
{
    L1 = ARM_L1;
    L2 = ARM_L2;
}

/**
  * @brief  逆运动学求解 (几何法)
  * @param  pos: 末端位置 (x, y, z) 单位mm
  * @param  joint: 输出关节角 (度)
  * @retval 0=成功, -1=超出工作空间
  */
int8_t arm_inverse_kinematic(arm_pos_t *pos, arm_joint_t *joint)
{
    if (!pos || !joint) return -1;
    
    float x = pos->x;
    float y = pos->y;
    float z = pos->z;
    
    /* 1. 计算yaw角 (底座旋转) */
    joint->yaw = atan2f(y, x) * 180.0f / PI;
    
    /* 2. 投影到臂平面 (忽略yaw，用x-z平面) */
    float r = sqrtf(x * x + y * y);  /* 水平距离 */
    float d = sqrtf(r * r + z * z);  /* 到原点距离 */
    
    /* 3. 检查工作空间 */
    if (d > (L1 + L2) || d < fabsf(L1 - L2)) {
        return -1;  /* 超出工作空间 */
    }
    
    /* 4. 余弦定理求肘关节角 */
    float cos_elbow = (L1 * L1 + L2 * L2 - d * d) / (2.0f * L1 * L2);
    if (cos_elbow < -1.0f) cos_elbow = -1.0f;
    if (cos_elbow > 1.0f) cos_elbow = 1.0f;
    
    joint->elbow = -acosf(cos_elbow) * 180.0f / PI;  /* 负号表示弯曲方向 */
    
    /* 5. 余弦定理求肩关节角 */
    float cos_shoulder = (L1 * L1 + d * d - L2 * L2) / (2.0f * L1 * d);
    if (cos_shoulder < -1.0f) cos_shoulder = -1.0f;
    if (cos_shoulder > 1.0f) cos_shoulder = 1.0f;
    
    float alpha = atan2f(z, r) * 180.0f / PI;
    float beta = acosf(cos_shoulder) * 180.0f / PI;
    joint->shoulder = alpha + beta;
    
    return 0;  /* 成功 */
}

/**
  * @brief  正运动学 (用于验证)
  * @param  joint: 关节角 (度)
  * @param  pos: 输出末端位置 (mm)
  */
void arm_forward_kinematic(arm_joint_t *joint, arm_pos_t *pos)
{
    if (!joint || !pos) return;
    
    float shoulder_rad = joint->shoulder * PI / 180.0f;
    float elbow_rad = joint->elbow * PI / 180.0f;
    float yaw_rad = joint->yaw * PI / 180.0f;
    
    /* 臂平面内坐标 */
    float r = L1 * cosf(shoulder_rad) + L2 * cosf(shoulder_rad + elbow_rad);
    float z = L1 * sinf(shoulder_rad) + L2 * sinf(shoulder_rad + elbow_rad);
    
    /* 转换到基座标系 */
    pos->x = r * cosf(yaw_rad);
    pos->y = r * sinf(yaw_rad);
    pos->z = z;
}

/**
  * @brief  关节限位
  * @param  joint: 关节角 (会被裁剪到限位范围内)
  */
void arm_joint_limit(arm_joint_t *joint)
{
    if (!joint) return;
    
    /* yaw限位 */
    if (joint->yaw < ARM_YAW_MIN) joint->yaw = ARM_YAW_MIN;
    if (joint->yaw > ARM_YAW_MAX) joint->yaw = ARM_YAW_MAX;
    
    /* shoulder限位 */
    if (joint->shoulder < ARM_SHOULDER_MIN) joint->shoulder = ARM_SHOULDER_MIN;
    if (joint->shoulder > ARM_SHOULDER_MAX) joint->shoulder = ARM_SHOULDER_MAX;
    
    /* elbow限位 */
    if (joint->elbow < ARM_ELBOW_MIN) joint->elbow = ARM_ELBOW_MIN;
    if (joint->elbow > ARM_ELBOW_MAX) joint->elbow = ARM_ELBOW_MAX;
    
    /* gripper限位 */
    if (joint->gripper < ARM_GRIPPER_MIN) joint->gripper = ARM_GRIPPER_MIN;
    if (joint->gripper > ARM_GRIPPER_MAX) joint->gripper = ARM_GRIPPER_MAX;
}

/**
  * @brief  关节平滑 (斜坡限幅)
  * @param  target: 目标角度
  * @param  current: 当前角度 (会被更新)
  * @param  step: 最大步进量 (度/周期)
  */
void arm_joint_smooth(arm_joint_t *target, arm_joint_t *current, float step)
{
    if (!target || !current) return;
    
    float diff;
    
    /* yaw平滑 */
    diff = target->yaw - current->yaw;
    if (diff > step) diff = step;
    if (diff < -step) diff = -step;
    current->yaw += diff;
    
    /* shoulder平滑 */
    diff = target->shoulder - current->shoulder;
    if (diff > step) diff = step;
    if (diff < -step) diff = -step;
    current->shoulder += diff;
    
    /* elbow平滑 */
    diff = target->elbow - current->elbow;
    if (diff > step) diff = step;
    if (diff < -step) diff = -step;
    current->elbow += diff;
    
    /* gripper平滑 */
    diff = target->gripper - current->gripper;
    if (diff > step) diff = step;
    if (diff < -step) diff = -step;
    current->gripper += diff;
}

/**
  * @brief  IK自检: 正逆运动学往返验证
  * @note   选若干测试点, IK(pos)->joints->FK(joints)->pos'
  *         比较 pos 和 pos' 的误差, 验证IK正确性
  *         通过ARM_DBG输出结果 (需arm_task.h中ARM_IK_SELFTEST_EN=1)
  */
void arm_ik_self_test(void)
{
    /* 测试点: 在工作空间内选若干位置 */
    /* 工作空间: |d| in [|L1-L2|, L1+L2] = [0, 300]mm */
    arm_pos_t test_pts[] = {
        {150.0f,   0.0f,   0.0f},   /* 正前方 */
        {200.0f,   0.0f, 100.0f},   /* 前上方 */
        {100.0f, 100.0f,   0.0f},   /* 右前方 */
        {200.0f, 200.0f,   0.0f},   /* 右侧远处 */
        { 50.0f,  50.0f,  50.0f},   /* 近处斜上 */
        {  0.0f,   0.0f, 250.0f},   /* 正上方 */
    };
    uint8_t num_tests = sizeof(test_pts) / sizeof(test_pts[0]);
    uint8_t pass_cnt = 0;

    ARM_DBG("\r\n=== IK Self Test ===\r\n");

    for (uint8_t i = 0; i < num_tests; i++) {
        arm_pos_t pos_in = test_pts[i];
        arm_joint_t joint;
        arm_pos_t pos_out;

        /* IK: pos -> joint */
        int8_t ret = arm_inverse_kinematic(&pos_in, &joint);
        if (ret != 0) {
            ARM_DBG("[T%d] IK FAIL (out of workspace): (%.1f, %.1f, %.1f)\r\n",
                    i, pos_in.x, pos_in.y, pos_in.z);
            continue;
        }

        /* FK: joint -> pos' */
        arm_forward_kinematic(&joint, &pos_out);

        /* 计算误差 */
        float err_x = pos_out.x - pos_in.x;
        float err_y = pos_out.y - pos_in.y;
        float err_z = pos_out.z - pos_in.z;
        float err_total = sqrtf(err_x*err_x + err_y*err_y + err_z*err_z);

        /* 判定: 误差 < 1mm 算通过 */
        uint8_t pass = (err_total < 1.0f) ? 1 : 0;
        if (pass) pass_cnt++;

        ARM_DBG("[T%d] %s in=(%.1f,%.1f,%.1f) j=(y=%.1f,s=%.1f,e=%.1f) "
                "out=(%.1f,%.1f,%.1f) err=%.3fmm\r\n",
                i, pass ? "PASS" : "FAIL",
                pos_in.x, pos_in.y, pos_in.z,
                joint.yaw, joint.shoulder, joint.elbow,
                pos_out.x, pos_out.y, pos_out.z,
                err_total);
    }

    ARM_DBG("=== IK Self Test: %d/%d passed ===\r\n\n", pass_cnt, num_tests);
}
