/**
  * @file    arm_ik.h
  * @brief   机械臂逆运动学 (4轴几何闭式解)
  * @note    底座yaw + 肩关节 + 肘关节 + 夹爪
  */

#ifndef __ARM_IK_H__
#define __ARM_IK_H__

#include <stdint.h>

/* 关节角度结构体 */
typedef struct {
    float yaw;       /* 底座偏航角 (度) */
    float shoulder;  /* 肩关节角 (度) */
    float elbow;     /* 肘关节角 (度) */
    float gripper;   /* 夹爪开合 (度) */
} arm_joint_t;

/* 末端位置结构体 */
typedef struct {
    float x;
    float y;
    float z;
} arm_pos_t;

/* 关节限位 (度, PD30S舵机270°范围) */
#define ARM_YAW_MIN        -135.0f
#define ARM_YAW_MAX         135.0f
#define ARM_SHOULDER_MIN   -135.0f
#define ARM_SHOULDER_MAX    135.0f
#define ARM_ELBOW_MIN      -135.0f
#define ARM_ELBOW_MAX       135.0f
#define ARM_GRIPPER_MIN      0.0f
#define ARM_GRIPPER_MAX     135.0f

/* 臂长参数 (mm, 根据实际测量修改) */
#define ARM_L1              150.0f   /* 大臂长度 */
#define ARM_L2              150.0f   /* 小臂长度 */

/* 函数声明 */
void arm_ik_init(void);
int8_t arm_inverse_kinematic(arm_pos_t *pos, arm_joint_t *joint);
void arm_forward_kinematic(arm_joint_t *joint, arm_pos_t *pos);
void arm_joint_limit(arm_joint_t *joint);
void arm_joint_smooth(arm_joint_t *target, arm_joint_t *current, float step);

/* IK自检: 正逆运动学往返验证 */
void arm_ik_self_test(void);

#endif
