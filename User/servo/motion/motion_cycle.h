#ifndef __MOTION_CYCLE_H__
#define __MOTION_CYCLE_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================= 配置 ================= */
#define MAX_CYCLE 6  // 最多同时存在循环数量，暴露给外部使用

/* ================= 类型定义 ================= */

/**
 * @brief Cycle状态回调函数类型
 * 
 * @param cycle_index cycle索引
 * @param loop_count 已完成的循环次数
 * @param max_loops 最大循环次数（0表示无限）
 * @param finished 是否已结束（完成所有循环）
 */
typedef void (*motion_cycle_status_cb_t)(uint32_t cycle_index, uint32_t loop_count, uint32_t max_loops, uint8_t finished);

/**
 * @brief Cycle创建配置结构体
 */
typedef struct {
    uint8_t* servo_ids;           // 舵机ID列表
    uint32_t servo_count;         // 舵机数量
    
    union {
        uint32_t** pose_list_pwm; // PWM姿态列表（二维数组）
        float** pose_list_angle;  // Angle姿态列表（二维数组）
    };
    
    uint32_t* pose_duration;      // 每个pose的运动时间（ms）
    uint32_t pose_count;          // pose数量
    uint32_t max_loops;           // 最大循环次数，0表示无限
    
    uint8_t mode;                 // 0=PWM模式，1=Angle模式
    void* user_data;              // 用户数据，用于协议层存储额外信息
} motion_cycle_config_t;

/**
 * @brief Cycle状态信息结构体
 */
typedef struct {
    bool     active;              // 是否激活
    bool     running;             // 是否正在运行
    uint8_t  current_pose_index;  // 当前姿态索引
    uint8_t  pose_count;          // 姿态总数
    uint32_t loop_count;          // 已完成的循环次数
    uint32_t max_loops;           // 最大循环次数
    uint32_t active_group_id;     // 当前活跃的motion_sync组ID
    void*    user_data;           // 用户数据
} motion_cycle_status_t;

/* ================= API ================= */

/**
 * @brief 创建一个 motion cycle
 *
 * @param config cycle配置
 * @param status_cb 状态回调函数（可为NULL）
 * @return >=0 cycle_index
 * @return <0  失败
 */
int32_t motion_cycle_create(const motion_cycle_config_t* config, motion_cycle_status_cb_t status_cb);

/**
 * @brief 启动一个 motion cycle
 *
 * @param cycle_index motion_cycle_create 返回的索引
 * @return 0 成功
 * @return <0 失败
 */
int32_t motion_cycle_start(uint32_t cycle_index);

/**
 * @brief 重新启动 motion cycle
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

/**
 * @brief 获取cycle状态
 *
 * @param cycle_index cycle索引
 * @param out_status 输出状态信息
 * @return true 成功
 * @return false 失败（cycle不存在）
 */
bool motion_cycle_get_status(uint32_t cycle_index, motion_cycle_status_t* out_status);

/**
 * @brief 设置cycle的用户数据
 *
 * @param cycle_index cycle索引
 * @param user_data 用户数据
 * @return true 成功
 * @return false 失败
 */
bool motion_cycle_set_user_data(uint32_t cycle_index, void* user_data);

/**
 * @brief 获取cycle的用户数据
 *
 * @param cycle_index cycle索引
 * @return 用户数据指针
 */
void* motion_cycle_get_user_data(uint32_t cycle_index);

/**
 * @brief 获取活跃cycle的数量
 *
 * @return 活跃cycle的数量
 */
uint32_t motion_cycle_get_active_count(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTION_CYCLE_H__ */
