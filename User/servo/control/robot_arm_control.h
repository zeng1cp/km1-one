#ifndef __ROBOT_ARM_CONTROL_H__
#define __ROBOT_ARM_CONTROL_H__

/*
    定义机械臂各个关节的舵机
    定义姿势数组
    播放姿势
*/

// 机械臂关节定义
typedef enum {
    ARM_JOINT_BASE = 0,      // 基座关节
    ARM_JOINT_SHOULDER,      // 肩关节
    ARM_JOINT_ELBOW,         // 肘关节
    ARM_JOINT_WRIST,         // 腕关节
    ARM_JOINT_WRIST_ROTATE,  // 腕部旋转
    // ARM_JOINT_GRIPPER,       // 夹爪
    ARM_JOINT_COUNT
} arm_joint_t;

// 机械臂姿态结构体
typedef struct {
    float joints[ARM_JOINT_COUNT];  // 各关节角度（度）
} arm_pose_t;

#endif /*__ROBOT_ARM_CONTROL_H__*/
