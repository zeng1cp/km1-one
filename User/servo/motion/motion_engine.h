#ifndef __MOTION_ENGINE_H__
#define __MOTION_ENGINE_H__
// 舵机运动引擎
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define MAX_SERVOS 6  // 根据您的硬件调整

// 运动完成回调函数类型
typedef void (*servo_motion_complete_cb_t)(uint8_t id);

// 舵机参数
typedef struct {
    uint32_t min_pwm_us;     // 最小pwm（单位：微秒）
    uint32_t mid_pwm_us;     // 中位pwm
    uint32_t max_pwm_us;     // 最大pwm
    float    min_angle_deg;  // 最小角度
    float    mid_angle_deg;  // 中位角度
    float    max_angle_deg;  // 最大角度
} servo_t;

// 舵机运动参数
typedef struct {
    servo_t  servo;        // 舵机参数
    uint32_t current_pwm;  // 当前PWM
    uint32_t target_pwm;   // 目标PWM
    uint32_t start_pwm;    // 起始PWM
    uint32_t steps_total;  // 总步数
    uint32_t steps_left;   // 剩余步数
    bool     is_moving;    // 是否在运动

    servo_motion_complete_cb_t complete_callback;  // 完成回调
} servo_motion_t;

// ==================== 初始化与配置 ====================
void    servo_motion_init(void);
void    servo_motion_set_params(uint8_t id, const servo_t* params);
servo_t servo_motion_get_params(uint8_t id);
void    servo_motion_set_global_complete_callback(servo_motion_complete_cb_t callback);

// ==================== 角度-PWM转换 ====================
uint32_t angle_to_pwm(uint8_t id, float angle_deg);
float    pwm_to_angle(uint8_t id, uint32_t pwm_us);

// ==================== 运动控制 ====================
void servo_move_angle(uint8_t id, float angle_deg, uint32_t duration_ms, servo_motion_complete_cb_t cb);
void servo_move_pwm(uint8_t id, uint32_t pwm_us, uint32_t duration_ms, servo_motion_complete_cb_t cb);
void servo_move_relative(uint8_t id, float delta_deg, uint32_t duration_ms, servo_motion_complete_cb_t cb);
void servo_move_home(uint8_t id, uint32_t duration_ms, servo_motion_complete_cb_t cb);
void servo_sync_to_hardware(void);

// ==================== 多舵机控制 ====================
void servo_move_angle_multiple(const uint8_t ids[],
                               const float   angles[],
                               uint8_t       count,
                               uint32_t      duration_ms,
                               servo_motion_complete_cb_t cb);
void servo_move_pwm_multiple(const uint8_t  ids[],
                             const uint32_t pwms[],
                             uint8_t        count,
                             uint32_t       duration_ms,
                             servo_motion_complete_cb_t cb);
// ==================== 状态控制 ====================
void servo_stop(uint8_t id);
void servo_pause(uint8_t id);
void servo_restart(uint8_t id);
void servo_stop_all(void);
void servo_emergency_stop(void);

// ==================== 状态查询 ====================
bool     servo_is_moving(uint8_t id);
bool     servo_any_moving(void);
uint32_t servo_get_current_pwm(uint8_t id);
float    servo_get_current_angle(uint8_t id);
float    servo_get_target_angle(uint8_t id);
uint32_t servo_get_moving_mask(void);  // 获取所有运动中舵机的掩码
uint32_t servo_get_remaining_time(uint8_t id);

// ==================== 掩码操作辅助函数 ====================
uint32_t servo_mask_from_ids(const uint8_t ids[], uint8_t count);

// ==================== 核心更新函数 ====================
void servo_motion_update_1ms(void);  // 在1ms定时器中断中调用

#endif /*__MOTION_ENGINE_H__*/
