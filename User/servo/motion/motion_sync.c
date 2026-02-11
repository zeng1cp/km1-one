#include "motion_sync.h"

#include <string.h>

#include "motion_engine.h"

#define MAX_SYNC_GROUPS  8
#define INVALID_GROUP_ID 0

/* ================= 内部数据结构 ================= */

typedef struct {
    uint32_t group_id;  // 0 = 空闲
} servo_usage_t;

typedef struct {
    uint32_t id;
    uint32_t mask;       // 本组舵机
    uint32_t done_mask;  // 已完成舵机
    bool     active;

    sync_group_complete_cb_t cb;
} sync_group_t;

/* ================= 静态变量 ================= */

static servo_usage_t servo_usage[MAX_SERVOS];
static sync_group_t  sync_groups[MAX_SYNC_GROUPS];
static uint32_t      next_group_id = 1;

/* ================= 私有函数声明 ================= */

static sync_group_t* find_free_group(void);
static sync_group_t* find_group(uint32_t group_id);
static void          on_servo_motion_complete(uint8_t servo_id);

/* ================= 初始化 ================= */

void motion_sync_init(void)
{
    memset(servo_usage, 0, sizeof(servo_usage));
    memset(sync_groups, 0, sizeof(sync_groups));
    next_group_id = 1;

    /* 注册舵机的完成回调 */
    servo_motion_set_global_complete_callback(on_servo_motion_complete);
}

void motion_sync_deinit(void)
{
    servo_emergency_stop();
    memset(servo_usage, 0, sizeof(servo_usage));
    memset(sync_groups, 0, sizeof(sync_groups));
}

/* ================= 私有工具 ================= */

static sync_group_t* find_free_group(void)
{
    for (int i = 0; i < MAX_SYNC_GROUPS; i++) {
        if (!sync_groups[i].active) return &sync_groups[i];
    }
    return NULL;
}

static sync_group_t* find_group(uint32_t group_id)
{
    for (int i = 0; i < MAX_SYNC_GROUPS; i++) {
        if (sync_groups[i].active && sync_groups[i].id == group_id) return &sync_groups[i];
    }
    return NULL;
}

/* ================= 舵机完成回调（核心） ================= */

static void on_servo_motion_complete(uint8_t servo_id)
{
    if (servo_id >= MAX_SERVOS) return;

    uint32_t gid = servo_usage[servo_id].group_id;
    if (gid == 0) return;

    sync_group_t* g = find_group(gid);
    if (!g || !g->active) return;

    g->done_mask |= (1U << servo_id);
    servo_usage[servo_id].group_id = 0;

    /* 判断组是否完成 */
    if (g->done_mask == g->mask) {
        g->active = false;

        if (g->cb) g->cb(g->id);
    }
}

/* ================= 舵机状态 ================= */

bool motion_sync_is_servo_available(uint8_t servo_id)
{
    if (servo_id >= MAX_SERVOS) return false;
    return servo_usage[servo_id].group_id == 0;
}

uint32_t motion_sync_get_busy_mask(void)
{
    uint32_t mask = 0;
    for (uint8_t i = 0; i < MAX_SERVOS; i++) {
        if (servo_usage[i].group_id != 0) mask |= (1U << i);
    }
    return mask;
}

uint32_t motion_sync_get_idle_mask(void)
{
    return ~motion_sync_get_busy_mask() & ((1U << MAX_SERVOS) - 1);
}

uint8_t motion_sync_get_busy_count(void)
{
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < MAX_SERVOS; i++) {
        if (servo_usage[i].group_id != 0) cnt++;
    }
    return cnt;
}

/* ================= 同步组控制 ================= */

uint32_t motion_sync_start_group(const uint8_t servo_ids[], uint8_t count)
{
    if (!servo_ids || count == 0) return INVALID_GROUP_ID;

    sync_group_t* g = find_free_group();
    if (!g) return INVALID_GROUP_ID;

    uint32_t mask = 0;

    for (uint8_t i = 0; i < count; i++) {
        uint8_t id = servo_ids[i];
        if (id >= MAX_SERVOS) return INVALID_GROUP_ID;
        if (!motion_sync_is_servo_available(id)) return INVALID_GROUP_ID;
        mask |= (1U << id);
    }

    g->id        = next_group_id++;
    g->mask      = mask;
    g->done_mask = 0;
    g->active    = true;
    g->cb        = NULL;

    for (uint8_t i = 0; i < count; i++) {
        servo_usage[servo_ids[i]].group_id = g->id;
    }

    return g->id;
}

bool motion_sync_release_group(uint32_t group_id)
{
    sync_group_t* g = find_group(group_id);
    if (!g) return false;

    for (uint8_t i = 0; i < MAX_SERVOS; i++) {
        if (g->mask & (1U << i)) {
            servo_stop(i);
            servo_usage[i].group_id = 0;
        }
    }

    g->active = false;
    return true;
}

bool motion_sync_restart_group(uint32_t group_id)
{
    sync_group_t* g = find_group(group_id);
    if (!g) return false;

    for (uint8_t i = 0; i < MAX_SERVOS; i++) {
        if (g->mask & (1U << i)) {
            servo_restart(i);
        }
    }
    return true;
}

bool motion_sync_pause_group(uint32_t group_id)
{
    sync_group_t* g = find_group(group_id);
    if (!g) return false;

    for (uint8_t i = 0; i < MAX_SERVOS; i++) {
        if (g->mask & (1U << i)) {
            servo_pause(i);
        }
    }
    return true;
}

bool motion_sync_is_group_complete(uint32_t group_id)
{
    sync_group_t* g = find_group(group_id);
    return (g == NULL) || (!g->active);
}

uint32_t motion_sync_get_group_mask(uint32_t group_id)
{
    sync_group_t* g = find_group(group_id);
    return g ? g->mask : 0;
}

/* ================= 高级接口 ================= */

uint32_t motion_sync_move_angle(const uint8_t            servo_ids[],
                                const float              angles[],
                                uint8_t                  count,
                                uint32_t                 duration_ms,
                                sync_group_complete_cb_t cb)
{
    uint32_t gid = motion_sync_start_group(servo_ids, count);
    if (gid == INVALID_GROUP_ID) return INVALID_GROUP_ID;

    sync_group_t* g = find_group(gid);
    if (g) g->cb = cb;

    for (uint8_t i = 0; i < count; i++) servo_move_angle(servo_ids[i], angles[i], duration_ms,NULL);

    return gid;
}

uint32_t motion_sync_move_pwm(const uint8_t            servo_ids[],
                              const uint32_t           pwms[],
                              uint8_t                  count,
                              uint32_t                 duration_ms,
                              sync_group_complete_cb_t cb)
{
    uint32_t gid = motion_sync_start_group(servo_ids, count);
    if (gid == INVALID_GROUP_ID) return INVALID_GROUP_ID;

    sync_group_t* g = find_group(gid);
    if (g) g->cb = cb;

    for (uint8_t i = 0; i < count; i++) servo_move_pwm(servo_ids[i], pwms[i], duration_ms,NULL);

    return gid;
}
