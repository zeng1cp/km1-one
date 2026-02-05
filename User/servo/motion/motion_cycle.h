#ifndef __MOTION_CYCLE_H__
#define __MOTION_CYCLE_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================= 配置 ================= */

/*
 * motion_cycle 是一个“调度层”
 * - 基于 motion_sync
 * - 支持多个 cycle 并行
 * - 每个 cycle 控制一组舵机
 */

/* ================= API ================= */

/**
 * @brief 创建一个 motion cycle
 *
 * @param servo_ids        舵机 ID 列表
 * @param servo_count      舵机数量
 * @param pose_list_pwm    PWM 姿态列表（二维数组），使用 Angle 模式时传 NULL
 *                         pose_list_pwm[i] -> 第 i 个 pose 的 pwm 数组
 * @param pose_list_angle  Angle 姿态列表（二维数组），使用 PWM 模式时传 NULL
 *                         pose_list_angle[i] -> 第 i 个 pose 的 angle 数组
 * @param pose_duration    每个 pose 的运动时间（ms）
 * @param pose_count       pose 数量
 * @param max_loops        最大循环次数，0 表示无限循环
 *
 * @return >=0 cycle_index
 * @return <0  失败
 */
int32_t motion_cycle_create(const uint8_t  servo_ids[],
                            uint32_t       servo_count,
                            uint32_t*      pose_list_pwm[],
                            float*         pose_list_angle[],
                            const uint32_t pose_duration[],
                            uint32_t       pose_count,
                            uint32_t       max_loops);

/**
 * @brief 启动一个 motion cycle
 *
 * @param cycle_index motion_cycle_create 返回的索引
 * @return 0 成功
 * @return <0 失败
 */
int32_t motion_cycle_start(uint32_t cycle_index);

/**
 * @brief c重新启动 motion cycle
 *
 * @param cycle_index motion_cycle_create 返回的索引
 * @return 0 成功
 * @return <0 失败
 */
int32_t motion_cycle_restart(uint32_t cycle_index);

/**
 * @brief 暂停一个 motion cycle（立即停止当前组）
 *
 * @param cycle_index cycle 索引
 * @return 0 成功
 * @return <0 失败
 */
int32_t motion_cycle_pause(uint32_t cycle_index);

/**
 * @brief 释放一个 motion cycle
 *
 * @param cycle_index cycle 索引
 * @return 0 成功
 * @return <0 失败
 *
 * @note 如果 cycle 正在运行，会先 stop 再释放
 */
int32_t motion_cycle_release(uint32_t cycle_index);

#ifdef __cplusplus
}
#endif

#endif /* __MOTION_CYCLE_H__ */
