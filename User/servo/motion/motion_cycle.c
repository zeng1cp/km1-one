#include "motion_cycle.h"
#include <string.h>
#include "motion_sync.h"

typedef struct {
    motion_cycle_config_t config;      // 配置信息
    uint32_t current_pose_index;       // 当前姿态索引
    bool     active;                   // 是否激活
    bool     running;                  // 是否正在运行
    uint32_t loop_count;               // 已完成的循环次数
    uint32_t active_group_id;          // 当前活跃的motion_sync组ID
    motion_cycle_status_cb_t status_cb; // 状态回调函数
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

static void motion_cycle_play_pose(motion_cycle_t* c, uint32_t cycle_index);

static void motion_cycle_on_group_done(uint32_t group_id)
{
    motion_cycle_t* c = find_cycle_by_group(group_id);
    if (!c || !c->running) return;

    // 找到cycle索引
    uint32_t cycle_index = 0;
    for (int i = 0; i < MAX_CYCLE; i++) {
        if (&cycle[i] == c) {
            cycle_index = i;
            break;
        }
    }

    c->current_pose_index++;

    if (c->current_pose_index >= c->config.pose_count) {
        c->current_pose_index = 0;
        c->loop_count++;

        // 调用状态回调
        if (c->status_cb) {
            c->status_cb(cycle_index, c->loop_count, c->config.max_loops, 0);
        }

        if (c->config.max_loops != 0 && c->loop_count >= c->config.max_loops) {
            c->running         = false;
            c->active_group_id = 0;
            
            // 循环结束，调用回调
            if (c->status_cb) {
                c->status_cb(cycle_index, c->loop_count, c->config.max_loops, 1);
            }
            return;
        }
    }

    motion_cycle_play_pose(c, cycle_index);
}

/* ========== 播放当前 pose（关键函数） ========== */

static void motion_cycle_play_pose(motion_cycle_t* c, uint32_t cycle_index)
{
    uint32_t idx      = c->current_pose_index;
    uint32_t duration = c->config.pose_duration[idx];

    if (c->config.mode == 0) { // PWM模式
        c->active_group_id = motion_sync_move_pwm(c->config.servo_ids,
                                                  c->config.pose_list_pwm[idx],
                                                  c->config.servo_count,
                                                  duration,
                                                  motion_cycle_on_group_done);
    } else { // Angle模式
        c->active_group_id = motion_sync_move_angle(c->config.servo_ids,
                                                    c->config.pose_list_angle[idx],
                                                    c->config.servo_count,
                                                    duration,
                                                    motion_cycle_on_group_done);
    }
    
    // 调用状态回调（开始新pose）
    if (c->status_cb) {
        c->status_cb(cycle_index, c->loop_count, c->config.max_loops, 0);
    }
}

/* ================= 创建循环 ================= */
int32_t motion_cycle_create(const motion_cycle_config_t* config, motion_cycle_status_cb_t status_cb)
{
    if (config == NULL) return -1;
    if (config->servo_ids == NULL) return -1;
    if (config->pose_duration == NULL) return -1;
    if (config->servo_count == 0 || config->pose_count == 0) return -1;

    int32_t idx = find_free_cycle();
    if (idx < 0) return -1;

    motion_cycle_t* c = &cycle[idx];
    memset(c, 0, sizeof(*c));

    // 复制配置（注意：这里只复制指针，不复制数据内容）
    memcpy(&c->config, config, sizeof(motion_cycle_config_t));
    c->status_cb = status_cb;
    c->active = true;
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
    c->running            = true;

    // 启动时调用状态回调
    if (c->status_cb) {
        c->status_cb(cycle_index, 0, c->config.max_loops, 0);
    }

    motion_cycle_play_pose(c, cycle_index);
    return 0;
}

int32_t motion_cycle_restart(uint32_t cycle_index)
{
    if (cycle_index >= MAX_CYCLE) return -1;

    motion_cycle_t* c = &cycle[cycle_index];
    if (!c->active) return -1;

    // 如果正在运行，先停止当前组
    if (c->running && c->active_group_id != 0) {
        motion_sync_release_group(c->active_group_id);
    }
    
    c->current_pose_index = 0;
    c->loop_count         = 0;
    c->active_group_id    = 0;
    c->running            = true;
    
    // 重启时调用状态回调
    if (c->status_cb) {
        c->status_cb(cycle_index, 0, c->config.max_loops, 0);
    }

    motion_cycle_play_pose(c, cycle_index);
    return 0;
}

int32_t motion_cycle_pause(uint32_t cycle_index)
{
    if (cycle_index >= MAX_CYCLE) return -1;

    motion_cycle_t* c = &cycle[cycle_index];
    if (!c->active || !c->running) return -1;

    motion_sync_pause_group(c->active_group_id);
    c->running = false;
    
    // 暂停时调用状态回调
    if (c->status_cb) {
        c->status_cb(cycle_index, c->loop_count, c->config.max_loops, 0);
    }
    
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
        
        // 释放时调用状态回调（标记为结束）
        if (c->status_cb) {
            c->status_cb(cycle_index, c->loop_count, c->config.max_loops, 1);
        }
    }

    memset(c, 0, sizeof(*c));
    return 0;
}

bool motion_cycle_get_status(uint32_t cycle_index, motion_cycle_status_t* out_status)
{
    if (cycle_index >= MAX_CYCLE) return false;
    
    motion_cycle_t* c = &cycle[cycle_index];
    if (!c->active) return false;
    
    out_status->active = c->active;
    out_status->running = c->running;
    out_status->current_pose_index = c->current_pose_index;
    out_status->pose_count = c->config.pose_count;
    out_status->loop_count = c->loop_count;
    out_status->max_loops = c->config.max_loops;
    out_status->active_group_id = c->active_group_id;
    out_status->user_data = c->config.user_data;
    
    return true;
}

bool motion_cycle_set_user_data(uint32_t cycle_index, void* user_data)
{
    if (cycle_index >= MAX_CYCLE) return false;
    
    motion_cycle_t* c = &cycle[cycle_index];
    if (!c->active) return false;
    
    c->config.user_data = user_data;
    return true;
}

void* motion_cycle_get_user_data(uint32_t cycle_index)
{
    if (cycle_index >= MAX_CYCLE) return NULL;
    
    motion_cycle_t* c = &cycle[cycle_index];
    if (!c->active) return NULL;
    
    return c->config.user_data;
}

uint32_t motion_cycle_get_active_count(void)
{
    uint32_t count = 0;
    for (int i = 0; i < MAX_CYCLE; i++) {
        if (cycle[i].active) {
            count++;
        }
    }
    return count;
}
