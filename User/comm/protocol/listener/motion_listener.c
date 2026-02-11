#include <stdio.h>

#include "motion_cycle.h"
#include "motion_engine.h"
#include "motion_sync.h"
#include "protocol.h"
#include "tinyframe/TinyFrame.h"

#ifndef MOTION_LISTENER_LOG_ENABLE
#define MOTION_LISTENER_LOG_ENABLE 1
#endif

#if MOTION_LISTENER_LOG_ENABLE
#define MOTION_LOG(fmt, ...) printf("[motion] " fmt "\n", ##__VA_ARGS__)
#define MOTION_DUMP(label, data, len)                            \
    do {                                                         \
        printf("[motion] %s (len=%u):", label, (unsigned)(len)); \
        for (uint16_t _i = 0; _i < (len); ++_i) {                \
            printf(" %02X", (unsigned)(data)[_i]);               \
        }                                                        \
        printf("\n");                                            \
    } while (0)
#else
#define MOTION_LOG(fmt, ...)          ((void)0)
#define MOTION_DUMP(label, data, len) ((void)0)
#endif

#define PROTO_CYCLE_MAX_SERVO MAX_SERVOS
#define PROTO_CYCLE_MAX_POSE  8

typedef struct {
    uint8_t  servo_ids[PROTO_CYCLE_MAX_SERVO];
    uint32_t servo_count;

    union {
        uint32_t pose_pwm[PROTO_CYCLE_MAX_POSE][PROTO_CYCLE_MAX_SERVO];
        float    pose_angle[PROTO_CYCLE_MAX_POSE][PROTO_CYCLE_MAX_SERVO];
    };
    uint32_t* pose_pwm_ptrs[PROTO_CYCLE_MAX_POSE];
    float*    pose_angle_ptrs[PROTO_CYCLE_MAX_POSE];

    uint32_t pose_duration[PROTO_CYCLE_MAX_POSE];
    uint32_t pose_count;

    uint8_t mode;       // 0=PWM, 1=Angle
    uint8_t allocated;  // 是否已分配
} proto_cycle_data_t;

static proto_cycle_data_t s_proto_cycle_data[MAX_CYCLE] = {0};
static uint32_t           s_cycle_notify_mask           = 0;

// 查找空闲的数据槽位
static int8_t find_free_proto_cycle_data(void)
{
    for (int i = 0; i < MAX_CYCLE; ++i) {
        if (s_proto_cycle_data[i].allocated == 0) {
            return (int8_t)i;
        }
    }
    return -1;
}

// 释放数据槽位
static void release_proto_cycle_data(uint8_t slot)
{
    if (slot >= MAX_CYCLE) return;
    s_proto_cycle_data[slot].allocated = 0;
}

static void protocol_cycle_status_cb(uint32_t cycle_index,
                                     uint32_t loop_count,
                                     uint32_t max_loops,
                                     uint8_t  finished)
{
    // 获取用户数据（协议数据）
    void* user_data = motion_cycle_get_user_data(cycle_index);
    if (user_data == NULL) {
        // 不是协议创建的cycle，不发送状态更新
        return;
    }

    // 转换为协议数据指针并检查是否已分配
    proto_cycle_data_t* pdata = (proto_cycle_data_t*)user_data;
    if (pdata->allocated == 0) {
        // 协议数据槽位未分配，可能是内部创建的cycle
        return;
    }

    // 检查通知掩码
    if (cycle_index >= 32) {
        return;
    }
    if ((s_cycle_notify_mask & (1U << cycle_index)) == 0) {
        return;
    }

    if (finished) {
        s_cycle_notify_mask &= ~(1U << cycle_index);
    }

    uint32_t remaining =
        (max_loops == 0) ? 0xFFFFFFFFu : (max_loops > loop_count ? (max_loops - loop_count) : 0);
    uint8_t resp[14];
    resp[0] = (uint8_t)MOTION_CMD_CYCLE_STATUS;
    proto_write_u32_le(resp, 1, cycle_index);
    proto_write_u32_le(resp, 5, loop_count);
    proto_write_u32_le(resp, 9, remaining);
    resp[13] = finished ? 1 : 0;
    protocol_send_state(STATE_CMD_MOTION, resp, (uint16_t)sizeof(resp));

    MOTION_LOG("Cycle status: index=%lu loop=%lu/%lu remaining=%lu finished=%u",
               (unsigned long)cycle_index,
               (unsigned long)loop_count,
               (unsigned long)max_loops,
               (unsigned long)remaining,
               (unsigned)finished);
}

static void protocol_motion_group_done(uint32_t group_id)
{
    uint8_t resp[6];
    resp[0] = (uint8_t)MOTION_CMD_STATUS;
    proto_write_u32_le(resp, 1, group_id);
    resp[5] = 1;
    protocol_send_state(STATE_CMD_MOTION, resp, (uint16_t)sizeof(resp));
}

TF_Result protocol_motion_listener(TinyFrame* tf, TF_Msg* msg)
{
    (void)tf;
    if (msg == NULL) {
        return TF_NEXT;
    }

    proto_cmd_view_t cmd_view;
    if (!proto_parse_cmd(msg->data, msg->len, &cmd_view)) {
        return TF_NEXT;
    }

    (void)protocol_motion_handle(cmd_view.cmd, cmd_view.payload, cmd_view.payload_len);
        return TF_STAY;
    }

bool protocol_motion_handle(uint8_t cmd, const uint8_t* payload, uint16_t len)
{
    // Payload format: [cmd][payload...]
    switch (cmd) {
        case MOTION_CMD_START: {
            MOTION_LOG("CMD START");
            MOTION_DUMP("payload", payload, len);
            // payload: [mode:u8][count:u8][duration:u32][ids...][values...]
            if (len < 6) {
                return false;
            }
            uint8_t  mode     = payload[0];  // 0=pwm(u32), 1=angle(f32)
            uint8_t  count    = payload[1];
            uint32_t duration = 0;
            if (!proto_read_u32_le(payload, len, 2, &duration)) {
                return false;
            }
            MOTION_LOG("mode=%u count=%u duration=%lu",
                       (unsigned)mode,
                       (unsigned)count,
                       (unsigned long)duration);
            uint16_t ids_off      = 6;
            uint16_t values_off   = (uint16_t)(ids_off + count);
            uint16_t value_size   = 4;
            uint16_t total_needed = (uint16_t)(values_off + (uint16_t)count * value_size);
            if (count == 0 || total_needed > len) {
                return false;
            }

            uint32_t gid = 0;
            if (mode == 0) {
                uint32_t pwms[MAX_SERVOS];
                if (count > MAX_SERVOS) {
                    return false;
                }
                for (uint8_t i = 0; i < count; ++i) {
                    if (!proto_read_u32_le(
                            payload, len, (uint16_t)(values_off + i * 4), &pwms[i])) {
                        return false;
                    }
                    MOTION_LOG("ids[%u]=%u pwm[%u]=%lu",
                               (unsigned)i,
                               (unsigned)payload[ids_off + i],
                               (unsigned)i,
                               (unsigned long)pwms[i]);
                }
                gid = motion_sync_move_pwm(
                    &payload[ids_off], pwms, count, duration, protocol_motion_group_done);
            } else if (mode == 1) {
                float angles[MAX_SERVOS];
                if (count > MAX_SERVOS) {
                    return false;
                }
                for (uint8_t i = 0; i < count; ++i) {
                    if (!proto_read_f32_le(
                            payload, len, (uint16_t)(values_off + i * 4), &angles[i])) {
                        return false;
                    }
                    MOTION_LOG("ids[%u]=%u angle[%u]=%.3f",
                               (unsigned)i,
                               (unsigned)payload[ids_off + i],
                               (unsigned)i,
                               (double)angles[i]);
                }
                gid = motion_sync_move_angle(
                    &payload[ids_off], angles, count, duration, protocol_motion_group_done);
            } else {
                return false;
            }

            uint8_t resp[5];
            resp[0] = (uint8_t)MOTION_CMD_START;
            proto_write_u32_le(resp, 1, gid);
            protocol_send_state(STATE_CMD_MOTION, resp, (uint16_t)sizeof(resp));
            return true;
        }
        case MOTION_CMD_STOP: {
            MOTION_LOG("CMD STOP");
            MOTION_DUMP("payload", payload, len);
            uint32_t gid = 0;
            if (!proto_read_u32_le(payload, len, 0, &gid)) {
                return false;
            }
            MOTION_LOG("group_id=%lu", (unsigned long)gid);
            return motion_sync_release_group(gid);
        }
        case MOTION_CMD_PAUSE: {
            MOTION_LOG("CMD PAUSE");
            MOTION_DUMP("payload", payload, len);
            uint32_t gid = 0;
            if (!proto_read_u32_le(payload, len, 0, &gid)) {
                return false;
            }
            MOTION_LOG("group_id=%lu", (unsigned long)gid);
            return motion_sync_pause_group(gid);
        }
        case MOTION_CMD_RESUME: {
            MOTION_LOG("CMD RESUME");
            MOTION_DUMP("payload", payload, len);
            uint32_t gid = 0;
            if (!proto_read_u32_le(payload, len, 0, &gid)) {
                return false;
            }
            MOTION_LOG("group_id=%lu", (unsigned long)gid);
            return motion_sync_restart_group(gid);
        }
        case MOTION_CMD_GET_STATUS: {
            MOTION_LOG("CMD GET_STATUS");
            MOTION_DUMP("payload", payload, len);
            uint32_t gid = 0;
            if (!proto_read_u32_le(payload, len, 0, &gid)) {
                return false;
            }
            MOTION_LOG("group_id=%lu", (unsigned long)gid);
            uint8_t resp[10];
            resp[0] = (uint8_t)MOTION_CMD_GET_STATUS;
            proto_write_u32_le(resp, 1, gid);
            proto_write_u32_le(resp, 5, motion_sync_get_group_mask(gid));
            resp[9] = (uint8_t)(motion_sync_is_group_complete(gid) ? 1 : 0);
            return protocol_send_state(STATE_CMD_MOTION, resp, (uint16_t)sizeof(resp));
        }
        case MOTION_CMD_SET_PLAN:
            MOTION_LOG("CMD SET_PLAN");
            MOTION_DUMP("payload", payload, len);
            return false;
        case MOTION_CMD_STATUS:
            MOTION_LOG("CMD STATUS");
            MOTION_DUMP("payload", payload, len);
            return true;
        case MOTION_CMD_CYCLE_CREATE: {
            MOTION_LOG("CMD CYCLE_CREATE");
            MOTION_DUMP("payload", payload, len);
            // payload: [mode:u8][servo_count:u8][pose_count:u8][max_loops:u32]
            //          [durations:u32 * pose_count][ids:u8 * servo_count]
            //          [values:pose_count * servo_count * 4]
            if (len < 7) {
                return false;
            }
            uint8_t  mode        = payload[0];  // 0=pwm(u32), 1=angle(f32)
            uint8_t  servo_count = payload[1];
            uint8_t  pose_count  = payload[2];
            uint32_t max_loops   = 0;
            if (!proto_read_u32_le(payload, len, 3, &max_loops)) {
                return false;
            }
            MOTION_LOG("mode=%u servo_count=%u pose_count=%u max_loops=%lu",
                       (unsigned)mode,
                       (unsigned)servo_count,
                       (unsigned)pose_count,
                       (unsigned long)max_loops);
            if (servo_count == 0 || pose_count == 0) {
                return false;
            }
            if (servo_count > PROTO_CYCLE_MAX_SERVO || pose_count > PROTO_CYCLE_MAX_POSE) {
                return false;
            }

            uint16_t durations_off = 7;
            uint16_t ids_off       = (uint16_t)(durations_off + (uint16_t)pose_count * 4U);
            uint16_t values_off    = (uint16_t)(ids_off + servo_count);
            uint16_t values_len    = (uint16_t)pose_count * (uint16_t)servo_count * 4U;
            uint16_t total_needed  = (uint16_t)(values_off + values_len);
            if (total_needed > len) {
                return false;
            }

            // 分配协议数据槽位
            int8_t data_slot = find_free_proto_cycle_data();
            if (data_slot < 0) {
                MOTION_LOG("No free protocol cycle data slot available");
                return false;
            }

            proto_cycle_data_t* pdata = &s_proto_cycle_data[data_slot];
            pdata->allocated          = 1;
            pdata->mode               = mode;
            pdata->servo_count        = servo_count;
            pdata->pose_count         = pose_count;

            // 复制舵机ID
            for (uint8_t i = 0; i < servo_count; ++i) {
                pdata->servo_ids[i] = payload[ids_off + i];
                MOTION_LOG("ids[%u]=%u", (unsigned)i, (unsigned)pdata->servo_ids[i]);
            }

            // 复制持续时间
            for (uint8_t p = 0; p < pose_count; ++p) {
                uint32_t dur = 0;
                if (!proto_read_u32_le(payload, len, (uint16_t)(durations_off + p * 4U), &dur)) {
                    release_proto_cycle_data((uint8_t)data_slot);
                    return false;
                }
                pdata->pose_duration[p] = dur;
                MOTION_LOG("durations[%u]=%lu", (unsigned)p, (unsigned long)dur);
            }

            // 准备数据指针并复制姿态数据
            for (uint8_t p = 0; p < pose_count; ++p) {
                if (mode == 0) {
                    pdata->pose_pwm_ptrs[p] = pdata->pose_pwm[p];
                    for (uint8_t i = 0; i < servo_count; ++i) {
                        uint32_t pwm = 0;
                        uint16_t off = (uint16_t)(values_off + (p * servo_count + i) * 4U);
                        if (!proto_read_u32_le(payload, len, off, &pwm)) {
                            release_proto_cycle_data((uint8_t)data_slot);
                            return false;
                        }
                        pdata->pose_pwm[p][i] = pwm;
                        MOTION_LOG(
                            "pose_pwm[%u][%u]=%lu", (unsigned)p, (unsigned)i, (unsigned long)pwm);
                    }
                } else if (mode == 1) {
                    pdata->pose_angle_ptrs[p] = pdata->pose_angle[p];
                    for (uint8_t i = 0; i < servo_count; ++i) {
                        float    angle = 0.0f;
                        uint16_t off   = (uint16_t)(values_off + (p * servo_count + i) * 4U);
                        if (!proto_read_f32_le(payload, len, off, &angle)) {
                            release_proto_cycle_data((uint8_t)data_slot);
                            return false;
                        }
                        pdata->pose_angle[p][i] = angle;
                        MOTION_LOG(
                            "pose_angle[%u][%u]=%.3f", (unsigned)p, (unsigned)i, (double)angle);
                    }
                }
            }

            // 创建 motion_cycle 配置
            motion_cycle_config_t config = {
                .servo_ids     = pdata->servo_ids,
                .servo_count   = servo_count,
                .pose_duration = pdata->pose_duration,
                .pose_count    = pose_count,
                .max_loops     = max_loops,
                .mode          = mode,
                .user_data     = pdata  // 保存协议数据指针
            };

            if (mode == 0) {
                config.pose_list_pwm = pdata->pose_pwm_ptrs;
            } else {
                config.pose_list_angle = pdata->pose_angle_ptrs;
            }

            // 创建 cycle
            int32_t cycle_index = motion_cycle_create(&config, protocol_cycle_status_cb);
            if (cycle_index < 0) {
                MOTION_LOG("motion_cycle_create failed");
                release_proto_cycle_data((uint8_t)data_slot);
                return false;
            }

            // 设置用户数据回指
            if (!motion_cycle_set_user_data(cycle_index, pdata)) {
                MOTION_LOG("motion_cycle_set_user_data failed");
                motion_cycle_release(cycle_index);
                release_proto_cycle_data((uint8_t)data_slot);
                return false;
            }

            // 设置通知掩码
            if (cycle_index < 32) {
                s_cycle_notify_mask |= (1U << cycle_index);
            }

            MOTION_LOG("CYCLE_CREATE success: index=%ld data_slot=%u",
                       (long)cycle_index,
                       (unsigned)data_slot);

            // 打印所有现有cycle状态
            for (uint8_t i = 0; i < MAX_CYCLE; ++i) {
                motion_cycle_status_t st;
                if (motion_cycle_get_status(i, &st)) {
                    MOTION_LOG("cycle[%u]: active=%u running=%u pose=%u/%u loops=%lu/%lu group=%lu",
                               (unsigned)i,
                               (unsigned)st.active,
                               (unsigned)st.running,
                               (unsigned)st.current_pose_index,
                               (unsigned)st.pose_count,
                               (unsigned long)st.loop_count,
                               (unsigned long)st.max_loops,
                               (unsigned long)st.active_group_id);
                }
            }

            // 返回所有现有cycle状态（复用CYCLE_LIST响应逻辑）
            uint8_t cycle_count = 0;
            uint8_t resp[256];
            resp[0]           = (uint8_t)MOTION_CMD_CYCLE_LIST;
            uint16_t resp_len = 2;
            for (uint8_t i = 0; i < MAX_CYCLE; ++i) {
                motion_cycle_status_t st;
                if (motion_cycle_get_status(i, &st)) {
                    if (resp_len + 18 > sizeof(resp)) {
                        MOTION_LOG("Response buffer overflow, stopping cycle enumeration");
                        break;
                    }
                    resp[resp_len + 0] = i;                                       // cycle index
                    resp[resp_len + 1] = st.active;                               // active
                    resp[resp_len + 2] = st.running;                              // running
                    resp[resp_len + 3] = st.current_pose_index;                   // current pose
                    resp[resp_len + 4] = st.pose_count;                           // pose count
                    proto_write_u32_le(resp, resp_len + 5, st.loop_count);        // loop count
                    proto_write_u32_le(resp, resp_len + 9, st.max_loops);         // max loops
                    proto_write_u32_le(resp, resp_len + 13, st.active_group_id);  // group id
                    resp_len += 17;
                    cycle_count++;
                }
            }
            resp[1] = cycle_count;
            MOTION_LOG("CYCLE_LIST (after create) count=%u total_len=%u",
                       (unsigned)cycle_count,
                       (unsigned)resp_len);
            return protocol_send_state(STATE_CMD_MOTION, resp, (uint16_t)resp_len);
        }
        case MOTION_CMD_CYCLE_START: {
            MOTION_LOG("CMD CYCLE_START");
            MOTION_DUMP("payload", payload, len);
            uint32_t idx = 0;
            if (!proto_read_u32_le(payload, len, 0, &idx)) {
                return false;
            }
            int result = motion_cycle_start(idx);
            if (result == 0) {
                MOTION_LOG("CYCLE_START success: index=%lu", (unsigned long)idx);
                motion_cycle_status_t st;
                if (motion_cycle_get_status(idx, &st)) {
                    MOTION_LOG(
                        "cycle[%lu]: active=%u running=%u pose=%u/%u loops=%lu/%lu group=%lu",
                        (unsigned long)idx,
                        (unsigned)st.active,
                        (unsigned)st.running,
                        (unsigned)st.current_pose_index,
                        (unsigned)st.pose_count,
                        (unsigned long)st.loop_count,
                        (unsigned long)st.max_loops,
                        (unsigned long)st.active_group_id);
                }
            }
            return result == 0;
        }
        case MOTION_CMD_CYCLE_RESTART: {
            MOTION_LOG("CMD CYCLE_RESTART");
            MOTION_DUMP("payload", payload, len);
            uint32_t idx = 0;
            if (!proto_read_u32_le(payload, len, 0, &idx)) {
                return false;
            }
            int result = motion_cycle_restart(idx);
            if (result == 0) {
                MOTION_LOG("CYCLE_RESTART success: index=%lu", (unsigned long)idx);
                motion_cycle_status_t st;
                if (motion_cycle_get_status(idx, &st)) {
                    MOTION_LOG(
                        "cycle[%lu]: active=%u running=%u pose=%u/%u loops=%lu/%lu group=%lu",
                        (unsigned long)idx,
                        (unsigned)st.active,
                        (unsigned)st.running,
                        (unsigned)st.current_pose_index,
                        (unsigned)st.pose_count,
                        (unsigned long)st.loop_count,
                        (unsigned long)st.max_loops,
                        (unsigned long)st.active_group_id);
                }
            }
            return result == 0;
        }
        case MOTION_CMD_CYCLE_PAUSE: {
            MOTION_LOG("CMD CYCLE_PAUSE");
            MOTION_DUMP("payload", payload, len);
            uint32_t idx = 0;
            if (!proto_read_u32_le(payload, len, 0, &idx)) {
                return false;
            }
            int result = motion_cycle_pause(idx);
            if (result == 0) {
                MOTION_LOG("CYCLE_PAUSE success: index=%lu", (unsigned long)idx);
                motion_cycle_status_t st;
                if (motion_cycle_get_status(idx, &st)) {
                    MOTION_LOG(
                        "cycle[%lu]: active=%u running=%u pose=%u/%u loops=%lu/%lu group=%lu",
                        (unsigned long)idx,
                        (unsigned)st.active,
                        (unsigned)st.running,
                        (unsigned)st.current_pose_index,
                        (unsigned)st.pose_count,
                        (unsigned long)st.loop_count,
                        (unsigned long)st.max_loops,
                        (unsigned long)st.active_group_id);
                }
            }
            return result == 0;
        }
        case MOTION_CMD_CYCLE_RELEASE: {
            MOTION_LOG("CMD CYCLE_RELEASE");
            MOTION_DUMP("payload", payload, len);
            uint32_t idx = 0;
            if (!proto_read_u32_le(payload, len, 0, &idx)) {
                return false;
            }

            // 获取用户数据（协议数据）
            proto_cycle_data_t* pdata = (proto_cycle_data_t*)motion_cycle_get_user_data(idx);
            if (pdata && pdata->allocated) {
                // 计算数据槽位索引
                int8_t data_slot = (int8_t)(pdata - s_proto_cycle_data);
                if (data_slot >= 0 && data_slot < MAX_CYCLE) {
                    release_proto_cycle_data((uint8_t)data_slot);
                    MOTION_LOG("Released protocol cycle data slot=%d for cycle=%lu",
                               data_slot,
                               (unsigned long)idx);
                }
            }

            // 清除通知掩码
            if (idx < 32) {
                s_cycle_notify_mask &= ~(1U << idx);
            }

            int result = motion_cycle_release(idx);
            if (result == 0) {
                MOTION_LOG("CYCLE_RELEASE success: index=%lu", (unsigned long)idx);
            }
            return result == 0;
        }
        case MOTION_CMD_CYCLE_GET_STATUS:
        case MOTION_CMD_CYCLE_STATUS: {
            MOTION_LOG("CMD CYCLE_GET_STATUS/STATUS");
            MOTION_DUMP("payload", payload, len);
            uint32_t idx = 0;
            if (!proto_read_u32_le(payload, len, 0, &idx)) {
                return false;
            }
            MOTION_LOG("cycle_index=%lu", (unsigned long)idx);
            motion_cycle_status_t st;
            if (!motion_cycle_get_status(idx, &st)) {
                return false;
            }
            MOTION_LOG(
                "active=%u running=%u current_pose=%u pose_count=%u loop_count=%lu max_loops=%lu "
                "active_group_id=%lu",
                (unsigned)st.active,
                (unsigned)st.running,
                (unsigned)st.current_pose_index,
                (unsigned)st.pose_count,
                (unsigned long)st.loop_count,
                (unsigned long)st.max_loops,
                (unsigned long)st.active_group_id);

            // 检查是否为协议创建的cycle
            proto_cycle_data_t* pdata = (proto_cycle_data_t*)st.user_data;
            if (pdata && pdata->allocated) {
                MOTION_LOG("cycle[%lu] is protocol cycle, mode=%u",
                           (unsigned long)idx,
                           (unsigned)pdata->mode);
            }

            uint8_t resp[21];
            resp[0] = (uint8_t)MOTION_CMD_CYCLE_GET_STATUS;
            proto_write_u32_le(resp, 1, idx);
            resp[5] = st.active;
            resp[6] = st.running;
            resp[7] = st.current_pose_index;
            resp[8] = st.pose_count;
            proto_write_u32_le(resp, 9, st.loop_count);
            proto_write_u32_le(resp, 13, st.max_loops);
            proto_write_u32_le(resp, 17, st.active_group_id);
            return protocol_send_state(STATE_CMD_MOTION, resp, (uint16_t)sizeof(resp));
        }
        case MOTION_CMD_CYCLE_LIST: {
            MOTION_LOG("CMD CYCLE_LIST");
            MOTION_DUMP("payload", payload, len);

            uint8_t cycle_count = 0;
            uint8_t resp[256];
            resp[0] = (uint8_t)MOTION_CMD_CYCLE_LIST;

            uint16_t resp_len = 2;  // cmd + count field

            // 遍历所有可能的cycle索引
            for (uint8_t i = 0; i < MAX_CYCLE; ++i) {
                motion_cycle_status_t st;
                if (motion_cycle_get_status(i, &st)) {
                    if (resp_len + 18 > sizeof(resp)) {
                        MOTION_LOG("Response buffer overflow, stopping cycle enumeration");
                        break;
                    }

                    resp[resp_len + 0] = i;                                       // cycle index
                    resp[resp_len + 1] = st.active;                               // active
                    resp[resp_len + 2] = st.running;                              // running
                    resp[resp_len + 3] = st.current_pose_index;                   // current pose
                    resp[resp_len + 4] = st.pose_count;                           // pose count
                    proto_write_u32_le(resp, resp_len + 5, st.loop_count);        // loop count
                    proto_write_u32_le(resp, resp_len + 9, st.max_loops);         // max loops
                    proto_write_u32_le(resp, resp_len + 13, st.active_group_id);  // group id
                    resp_len += 17;
                    cycle_count++;

                    // 检查是否为协议创建的cycle
                    proto_cycle_data_t* pdata = (proto_cycle_data_t*)st.user_data;
                    if (pdata && pdata->allocated) {
                        MOTION_LOG(
                            "cycle[%u]: protocol_cycle mode=%u active=%u running=%u pose=%u/%u "
                            "loops=%lu/%lu group=%lu",
                            (unsigned)i,
                            (unsigned)pdata->mode,
                            (unsigned)st.active,
                            (unsigned)st.running,
                            (unsigned)st.current_pose_index,
                            (unsigned)st.pose_count,
                            (unsigned long)st.loop_count,
                            (unsigned long)st.max_loops,
                            (unsigned long)st.active_group_id);
                    } else {
                        MOTION_LOG(
                            "cycle[%u]: internal_cycle active=%u running=%u pose=%u/%u "
                            "loops=%lu/%lu group=%lu",
                            (unsigned)i,
                            (unsigned)st.active,
                            (unsigned)st.running,
                            (unsigned)st.current_pose_index,
                            (unsigned)st.pose_count,
                            (unsigned long)st.loop_count,
                            (unsigned long)st.max_loops,
                            (unsigned long)st.active_group_id);
                    }
                }
            }

            resp[1] = cycle_count;
            MOTION_LOG(
                "CYCLE_LIST count=%u total_len=%u", (unsigned)cycle_count, (unsigned)resp_len);
            return protocol_send_state(STATE_CMD_MOTION, resp, (uint16_t)resp_len);
        }
        default:
            return false;
    }
}
