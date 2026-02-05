#include "motion_engine.h"

#include <string.h>

#include "../drivers/servo_hal.h"  // 硬件层

// 全局舵机运动数组
static servo_motion_t servo_motions[MAX_SERVOS];

// 全局运动状态掩码
static uint32_t global_moving_mask = 0;

// 全局完成回调（用于同步管理器）
static servo_motion_complete_cb_t global_complete_callback = NULL;

// 处理单个舵机运动完成
static void on_servo_complete(uint8_t servo_id)
{
    servo_motion_t* sm = &servo_motions[servo_id];

    // 清除全局运动掩码中的对应位
    global_moving_mask &= ~(1 << servo_id);

    // 1. 先调用舵机自己的回调（如果设置了）
    if (sm->complete_callback != NULL) {
        sm->complete_callback(servo_id);
        // 注意：调用后不清除，以便复用
    }

    // 2. 再调用全局回调（用于同步管理器）
    if (global_complete_callback != NULL) {
        global_complete_callback(servo_id);
    }
}

// ==================== 私有辅助函数 ====================

// 根据ID数组创建舵机掩码
static uint32_t create_servo_mask(const uint8_t ids[], uint8_t count)
{
    uint32_t mask = 0;
    for (uint8_t i = 0; i < count; i++) {
        if (ids[i] < MAX_SERVOS) {
            mask |= (1 << ids[i]);
        }
    }
    return mask;
}

// ==================== 初始化与配置 ====================

/**
 * @brief 初始化时并未输出硬件PWM,需要调用servo_sync_to_hardware舵机才会运动到中位，防止在上电时舵机突然的运动
 *
 */
void servo_motion_init(void)
{
    // 默认270度舵机参数（0-270度对应500-2500us）
    servo_t servo_default = {.min_pwm_us    = 500,
                             .mid_pwm_us    = 1500,
                             .max_pwm_us    = 2500,
                             .min_angle_deg = 0.0f,
                             .mid_angle_deg = 135.0f,
                             .max_angle_deg = 270.0f};

    for (int i = 0; i < MAX_SERVOS; i++) {
        servo_motion_t* sm = &servo_motions[i];

        // 使用memcpy复制默认参数
        memcpy(&(sm->servo), &servo_default, sizeof(servo_t));

        // 初始化运动状态
        sm->current_pwm = sm->servo.mid_pwm_us;
        sm->target_pwm  = sm->servo.mid_pwm_us;
        sm->start_pwm   = sm->servo.mid_pwm_us;
        sm->steps_total = 0;
        sm->steps_left  = 0;
        sm->is_moving   = false;

        sm->complete_callback = NULL;
        // servo_hal_set_pwm(i, sm->current_pwm);
    }

    global_moving_mask = 0;

    global_complete_callback = NULL;
    // 注意：硬件初始化 servo_hal_init() 由主程序调用
}

void servo_motion_set_params(uint8_t id, const servo_t* params)
{
    if (id >= MAX_SERVOS || params == NULL) return;

    servo_motion_t* sm = &servo_motions[id];

    // 复制新的参数
    memcpy(&(sm->servo), params, sizeof(servo_t));

    // 确保参数有效性
    if (sm->servo.min_pwm_us >= sm->servo.max_pwm_us) {
        // 恢复默认值
        sm->servo.min_pwm_us = 500;
        sm->servo.mid_pwm_us = 1500;
        sm->servo.max_pwm_us = 2500;
    }

    // 重新计算中位角度（如果未设置）
    if (sm->servo.mid_angle_deg < sm->servo.min_angle_deg
        || sm->servo.mid_angle_deg > sm->servo.max_angle_deg) {
        sm->servo.mid_angle_deg = (sm->servo.min_angle_deg + sm->servo.max_angle_deg) / 2.0f;
    }
}

servo_t servo_motion_get_params(uint8_t id)
{
    static servo_t default_params = {500, 1500, 2500, 0.0f, 135.0f, 270.0f};

    if (id >= MAX_SERVOS) return default_params;
    return servo_motions[id].servo;
}

void servo_motion_set_complete_callback(uint8_t id, servo_motion_complete_cb_t callback)
{
    if (id >= MAX_SERVOS) return;
    servo_motions[id].complete_callback = callback;
}

// 设置全局完成回调
void servo_motion_set_global_complete_callback(servo_motion_complete_cb_t callback)
{
    global_complete_callback = callback;
}

// ==================== 角度-PWM转换 ====================

uint32_t angle_to_pwm(uint8_t id, float angle_deg)
{
    if (id >= MAX_SERVOS) return 1500;

    servo_motion_t* sm = &servo_motions[id];
    const servo_t*  s  = &sm->servo;

    // 角度限制
    if (angle_deg < s->min_angle_deg) {
        angle_deg = s->min_angle_deg;
    } else if (angle_deg > s->max_angle_deg) {
        angle_deg = s->max_angle_deg;
    }

    // 计算角度比例
    float angle_range = s->max_angle_deg - s->min_angle_deg;
    if (angle_range < 0.001f) return s->mid_pwm_us;  // 避免除零

    float ratio = (angle_deg - s->min_angle_deg) / angle_range;

    // 限制比例在0-1之间
    if (ratio < 0.0f) ratio = 0.0f;
    if (ratio > 1.0f) ratio = 1.0f;

    // 计算PWM值
    uint32_t pwm_range = s->max_pwm_us - s->min_pwm_us;
    uint32_t pwm       = s->min_pwm_us + (uint32_t)(ratio * pwm_range);

    return pwm;
}

float pwm_to_angle(uint8_t id, uint32_t pwm_us)
{
    if (id >= MAX_SERVOS) return 135.0f;  // 默认返回中位角度

    servo_motion_t* sm = &servo_motions[id];
    const servo_t*  s  = &sm->servo;

    // PWM限制
    if (pwm_us < s->min_pwm_us) pwm_us = s->min_pwm_us;
    if (pwm_us > s->max_pwm_us) pwm_us = s->max_pwm_us;

    // 计算PWM比例
    float pwm_range = (float)(s->max_pwm_us - s->min_pwm_us);
    if (pwm_range < 0.001f) return s->mid_angle_deg;  // 避免除零

    float ratio = (float)(pwm_us - s->min_pwm_us) / pwm_range;

    // 计算角度
    float angle_range = s->max_angle_deg - s->min_angle_deg;
    float angle       = s->min_angle_deg + (ratio * angle_range);

    return angle;
}

// ==================== 运动控制 ====================

void servo_move_pwm(uint8_t id, uint32_t pwm_us, uint32_t duration_ms)
{
    if (id >= MAX_SERVOS) return;

    servo_motion_t* sm = &servo_motions[id];
    const servo_t*  s  = &sm->servo;

    // PWM边界检查
    if (pwm_us < s->min_pwm_us) pwm_us = s->min_pwm_us;
    if (pwm_us > s->max_pwm_us) pwm_us = s->max_pwm_us;

    // 如果已经在目标位置，直接返回
    if (pwm_us == sm->current_pwm) {
        sm->is_moving = false;
        return;
    }

    // 设置运动参数
    sm->start_pwm   = sm->current_pwm;
    sm->target_pwm  = pwm_us;
    sm->steps_total = duration_ms;
    sm->steps_left  = duration_ms;
    sm->is_moving   = true;

    // 设置全局运动掩码
    global_moving_mask |= (1 << id);
}

void servo_move_angle(uint8_t id, float angle_deg, uint32_t duration_ms)
{
    if (id >= MAX_SERVOS) return;
    // 转换为PWM
    uint32_t target_pwm = angle_to_pwm(id, angle_deg);
    // 调用PWM版本，复用所有检查和处理逻辑
    servo_move_pwm(id, target_pwm, duration_ms);
}

void servo_move_relative(uint8_t id, float delta_deg, uint32_t duration_ms)
{
    if (id >= MAX_SERVOS) return;

    // 获取当前角度
    float current_angle = servo_get_current_angle(id);
    float target_angle  = current_angle + delta_deg;

    // 移动到新角度
    servo_move_angle(id, target_angle, duration_ms);
}

void servo_move_home(uint8_t id, uint32_t duration_ms)
{
    if (id >= MAX_SERVOS) return;

    // 移动到中位角度
    servo_motion_t* sm = &servo_motions[id];
    servo_move_angle(id, sm->servo.mid_angle_deg, duration_ms);
}

// 输出 current_pwm 到 PWM 硬件
void servo_sync_to_hardware(void)
{
    for (uint8_t i = 0; i < MAX_SERVOS; i++) {
        servo_motion_t* sm = &servo_motions[i];
        servo_hal_set_pwm(i, sm->current_pwm);
    }
}

// ==================== 多舵机控制 ====================

// 角度控制版本
void servo_move_angle_multiple(const uint8_t ids[],
                               const float   angles[],
                               uint8_t       count,
                               uint32_t      duration_ms)
{
    if (ids == NULL || angles == NULL || count == 0) return;

    for (uint8_t i = 0; i < count; i++) {
        servo_move_angle(ids[i], angles[i], duration_ms);
    }
}

// PWM控制版本
void servo_move_pwm_multiple(const uint8_t  ids[],
                             const uint32_t pwms[],
                             uint8_t        count,
                             uint32_t       duration_ms)
{
    if (ids == NULL || pwms == NULL || count == 0) return;

    for (uint8_t i = 0; i < count; i++) {
        servo_move_pwm(ids[i], pwms[i], duration_ms);
    }
}

// ==================== 状态控制 ====================

void servo_stop(uint8_t id)
{
    if (id >= MAX_SERVOS) return;

    servo_motion_t* sm = &servo_motions[id];

    // 如果舵机在运动，停止它
    if (sm->is_moving) {
        sm->is_moving  = false;
        sm->steps_left = 0;

        // 手动触发完成处理
        on_servo_complete(id);
    }
}

void servo_pause(uint8_t id)
{
    if (id >= MAX_SERVOS) return;

    servo_motion_t* sm = &servo_motions[id];

    // 如果舵机在运动，暂停它
    if (sm->is_moving) {
        sm->is_moving = false;
    }
}

void servo_restart(uint8_t id)
{
    if (id >= MAX_SERVOS) return;

    servo_motion_t* sm = &servo_motions[id];

    // 如果舵机在运动，停止它
    if (!sm->is_moving) {
        sm->is_moving = true;
    }
}

void servo_stop_all(void)
{
    for (uint8_t i = 0; i < MAX_SERVOS; i++) {
        servo_stop(i);
    }
}

void servo_emergency_stop(void)
{
    for (uint8_t i = 0; i < MAX_SERVOS; i++) {
        servo_motion_t* sm = &servo_motions[i];

        // 立即停止
        sm->is_moving  = false;
        sm->steps_left = 0;

        // 快速设置到中位（安全位置）
        sm->current_pwm = sm->servo.mid_pwm_us;
        servo_hal_set_pwm(i, sm->servo.mid_pwm_us);
    }
}

// ==================== 状态查询 ====================

bool servo_is_moving(uint8_t id)
{
    if (id >= MAX_SERVOS) return false;
    return servo_motions[id].is_moving;
}

bool servo_any_moving(void)
{
    return global_moving_mask != 0;
}

uint32_t servo_get_moving_mask(void)
{
    return global_moving_mask;
}

uint32_t servo_get_current_pwm(uint8_t id)
{
    if (id >= MAX_SERVOS) return 1500;
    return servo_motions[id].current_pwm;
}

float servo_get_current_angle(uint8_t id)
{
    if (id >= MAX_SERVOS) return 135.0f;
    return pwm_to_angle(id, servo_motions[id].current_pwm);
}

float servo_get_target_angle(uint8_t id)
{
    if (id >= MAX_SERVOS) return 135.0f;
    return pwm_to_angle(id, servo_motions[id].target_pwm);
}

uint32_t servo_get_remaining_time(uint8_t id)
{
    if (id >= MAX_SERVOS) return 0;
    return servo_motions[id].steps_left;
}

// ==================== 掩码操作辅助函数 ====================

uint32_t servo_mask_from_ids(const uint8_t ids[], uint8_t count)
{
    return create_servo_mask(ids, count);
}

// ==================== 核心更新函数 ====================

// 缓动函数：三次缓动（平滑）
static float ease_in_out_cubic(float t)
{
    t = powf(t, 0.5f);
    return (t * t * (3.0f - 2.0f * t));
}

void servo_motion_update_1ms(void)
{
    for (uint8_t i = 0; i < MAX_SERVOS; i++) {
        servo_motion_t* sm = &servo_motions[i];

        if (!sm->is_moving) continue;

        // 减少剩余步数
        if (sm->steps_left > 0) {
            sm->steps_left--;
        }

        // 检查是否完成
        if (sm->steps_left == 0) {
            sm->current_pwm = sm->target_pwm;
            sm->is_moving   = false;
            servo_hal_set_pwm(i, sm->target_pwm);

            // 处理完成
            on_servo_complete(i);

            continue;
        }

        // 计算进度并更新PWM
        float t = 1.0f - ((float)sm->steps_left / sm->steps_total);
        t       = ease_in_out_cubic(t);

        // 使用有符号整数计算PWM范围
        int32_t pwm_range   = (int32_t)sm->target_pwm - (int32_t)sm->start_pwm;
        int32_t new_pwm_int = (int32_t)sm->start_pwm + (int32_t)(pwm_range * t);

        // 确保PWM值不为负
        uint32_t new_pwm = (new_pwm_int < 0) ? 0 : (uint32_t)new_pwm_int;

        if (new_pwm != sm->current_pwm) {
            sm->current_pwm = new_pwm;
            servo_hal_set_pwm(i, new_pwm);
        }
    }
}
