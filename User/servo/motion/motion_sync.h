#ifndef __MOTION_SYNC_H__
#define __MOTION_SYNC_H__
#include <stdbool.h>
#include <stdint.h>

typedef void (*sync_group_complete_cb_t)(uint32_t group_id);

/* 初始化 / 反初始化 */
void motion_sync_init(void);
void motion_sync_deinit(void);

/* ================= 舵机状态查询 ================= */

bool     motion_sync_is_servo_available(uint8_t servo_id);
uint32_t motion_sync_get_busy_mask(void);
uint32_t motion_sync_get_idle_mask(void);
uint8_t  motion_sync_get_busy_count(void);

/* ================= 同步组控制 ================= */

uint32_t motion_sync_start_group(const uint8_t servo_ids[], uint8_t count);
bool     motion_sync_restart_group(uint32_t group_id);
bool     motion_sync_pause_group(uint32_t group_id);
bool     motion_sync_release_group(uint32_t group_id);
bool     motion_sync_is_group_complete(uint32_t group_id);
uint32_t motion_sync_get_group_mask(uint32_t group_id);

/* ================= 高级接口（直接运动） ================= */

uint32_t motion_sync_move_angle(const uint8_t            servo_ids[],
                                const float              angles[],
                                uint8_t                  count,
                                uint32_t                 duration_ms,
                                sync_group_complete_cb_t cb);

uint32_t motion_sync_move_pwm(const uint8_t            servo_ids[],
                              const uint32_t           pwms[],
                              uint8_t                  count,
                              uint32_t                 duration_ms,
                              sync_group_complete_cb_t cb);

#endif /*__MOTION_SYNC_H__*/
