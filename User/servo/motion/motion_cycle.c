#include "motion_cycle.h"

#include <string.h>

#include "motion_sync.h"

#define MAX_CYCLE 5  // 最多同时存在循环数量

typedef struct {
    uint8_t* servo_ids;
    uint32_t servo_count;

    uint32_t** pose_list_pwm;
    float**    pose_list_angle;
    uint32_t*  pose_duration;
    uint32_t   pose_count;

    uint32_t current_pose_index;

    bool     active;
    bool     running;
    uint32_t loop_count;
    uint32_t max_loops;  // 0 = 无限

    uint32_t active_group_id;  // ★ 关键：当前 motion_sync group
} motion_cycle_t;

static motion_cycle_t cycle[MAX_CYCLE] = {0};

/* ================= 私有工具 ================= */

static motion_cycle_t* find_cycle_by_group(uint32_t group_id)
{
    for (int i = 0; i < MAX_CYCLE; i++) {
        if (cycle[i].active && cycle[i].running && cycle[i].active_group_id == group_id) {
            return &cycle[i];
        }
    }
    return NULL;
}

static int32_t find_free_cycle(void)
{
    for (int i = 0; i < MAX_CYCLE; i++) {
        if (!cycle[i].active) return i;
    }
    return -1;
}

/* ======= motion_sync 回调（核心状态机） ======= */

static void motion_cycle_play_pose(motion_cycle_t* c);

static void motion_cycle_on_group_done(uint32_t group_id)
{
    motion_cycle_t* c = find_cycle_by_group(group_id);
    if (!c || !c->running) return;

    c->current_pose_index++;

    if (c->current_pose_index >= c->pose_count) {
        c->current_pose_index = 0;
        c->loop_count++;

        if (c->max_loops != 0 && c->loop_count >= c->max_loops) {
            c->running         = false;
            c->active_group_id = 0;
            return;
        }
    }

    motion_cycle_play_pose(c);
}

/* ========== 播放当前 pose（关键函数） ========== */

static void motion_cycle_play_pose(motion_cycle_t* c)
{
    uint32_t idx      = c->current_pose_index;
    uint32_t duration = c->pose_duration[idx];

    if (c->pose_list_pwm) {
        c->active_group_id = motion_sync_move_pwm(c->servo_ids,
                                                  c->pose_list_pwm[idx],
                                                  c->servo_count,
                                                  duration,
                                                  motion_cycle_on_group_done);
    } else {
        c->active_group_id = motion_sync_move_angle(c->servo_ids,
                                                    c->pose_list_angle[idx],
                                                    c->servo_count,
                                                    duration,
                                                    motion_cycle_on_group_done);
    }
}

/* ================= 创建循环 ================= */
// 创建成功返回cycle_index
int32_t motion_cycle_create(const uint8_t  servo_ids[],
                            uint32_t       servo_count,
                            uint32_t*      pose_list_pwm[],
                            float*         pose_list_angle[],
                            const uint32_t pose_duration[],
                            uint32_t       pose_count,
                            uint32_t       max_loops)
{
    int32_t idx = find_free_cycle();
    if (idx < 0) return -1;

    motion_cycle_t* c = &cycle[idx];
    memset(c, 0, sizeof(*c));

    c->servo_ids       = (uint8_t*)servo_ids;
    c->servo_count     = servo_count;
    c->pose_list_pwm   = pose_list_pwm;
    c->pose_list_angle = pose_list_angle;
    c->pose_duration   = (uint32_t*)pose_duration;
    c->pose_count      = pose_count;
    c->max_loops       = max_loops;

    c->active  = true;
    c->running = false;

    return idx;
}

int32_t motion_cycle_start(uint32_t cycle_index)
{
    if (cycle_index >= MAX_CYCLE) return -1;

    motion_cycle_t* c = &cycle[cycle_index];
    if (!c->active || c->running) return -1;

    c->current_pose_index = 0;
    c->loop_count         = 0;
    c->active_group_id    = 0;

    c->running = true;

    motion_cycle_play_pose(c);
    return 0;
}

int32_t motion_cycle_restart(uint32_t cycle_index)
{
    if (cycle_index >= MAX_CYCLE) return -1;

    motion_cycle_t* c = &cycle[cycle_index];
    if (!c->active || c->running) return -1;

    motion_sync_restart_group(c->active_group_id);
    c->running = true;
    return 0;
}

int32_t motion_cycle_pause(uint32_t cycle_index)
{
    if (cycle_index >= MAX_CYCLE) return -1;

    motion_cycle_t* c = &cycle[cycle_index];
    if (!c->active || !c->running) return -1;

    motion_sync_pause_group(c->active_group_id);
    c->running = false;
    return 0;
}

int32_t motion_cycle_release(uint32_t cycle_index)
{
    if (cycle_index >= MAX_CYCLE) return -1;

    motion_cycle_t* c = &cycle[cycle_index];
    if (!c->active) return -1;

    if (c->running) {
        motion_sync_release_group(c->active_group_id);
        c->active_group_id = 0;
        c->running         = false;
    }

    memset(c, 0, sizeof(*c));
    return 0;
}
