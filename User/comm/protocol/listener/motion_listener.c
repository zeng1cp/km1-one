#include <stdio.h>
#include <string.h>

#include "motion_cycle.h"
#include "motion_engine.h"
#include "motion_sync.h"
#include "motion_codec.h"
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
    uint8_t allocated;  // allocated flag
} proto_cycle_data_t;

static proto_cycle_data_t s_proto_cycle_data[MAX_CYCLE] = {0};
static uint32_t           s_cycle_notify_mask           = 0;
static bool send_motion_payload(const uint8_t* payload, uint16_t payload_len)
{
    return protocol_send_state(STATE_CMD_MOTION, payload, payload_len);
}

static bool encode_and_send_motion_start(uint32_t group_id)
{
    uint8_t payload[5];
    proto_motion_start_resp_t resp = {
        .subcmd = (uint8_t)MOTION_CMD_START,
        .group_id = group_id,
    };

    uint16_t payload_len = proto_encode_motion_start_resp(&resp, payload, (uint16_t)sizeof(payload));
    if (payload_len == 0U) {
        return false;
    }
    return send_motion_payload(payload, payload_len);
}

static bool encode_and_send_motion_status(uint8_t subcmd, uint32_t group_id, uint8_t complete)
{
    uint8_t payload[6];
    proto_motion_status_resp_t resp = {
        .subcmd = subcmd,
        .group_id = group_id,
        .complete = complete,
    };

    uint16_t payload_len = proto_encode_motion_status_resp(&resp, payload, (uint16_t)sizeof(payload));
    if (payload_len == 0U) {
        return false;
    }
    return send_motion_payload(payload, payload_len);
}

static bool encode_and_send_motion_get_status(uint32_t group_id, uint32_t mask, uint8_t complete)
{
    uint8_t payload[10];
    proto_motion_get_status_resp_t resp = {
        .subcmd = (uint8_t)MOTION_CMD_GET_STATUS,
        .group_id = group_id,
        .mask = mask,
        .complete = complete,
    };

    uint16_t payload_len =
        proto_encode_motion_get_status_resp(&resp, payload, (uint16_t)sizeof(payload));
    if (payload_len == 0U) {
        return false;
    }
    return send_motion_payload(payload, payload_len);
}

static bool encode_and_send_cycle_status_update(uint32_t cycle_index,
                                                uint32_t loop_count,
                                                uint32_t remaining,
                                                uint8_t finished)
{
    uint8_t payload[14];
    proto_motion_cycle_status_update_resp_t resp = {
        .subcmd = (uint8_t)MOTION_CMD_CYCLE_STATUS,
        .cycle_index = cycle_index,
        .loop_count = loop_count,
        .remaining = remaining,
        .finished = finished,
    };

    uint16_t payload_len = proto_encode_motion_cycle_status_update_resp(
        &resp, payload, (uint16_t)sizeof(payload));
    if (payload_len == 0U) {
        return false;
    }
    return send_motion_payload(payload, payload_len);
}

static bool encode_and_send_cycle_status(uint32_t cycle_index, const motion_cycle_status_t* st)
{
    uint8_t payload[21];
    proto_motion_cycle_status_resp_t resp = {
        .subcmd = (uint8_t)MOTION_CMD_CYCLE_GET_STATUS,
        .cycle_index = cycle_index,
        .active = (uint8_t)st->active,
        .running = (uint8_t)st->running,
        .current_pose = st->current_pose_index,
        .pose_count = st->pose_count,
        .loop_count = st->loop_count,
        .max_loops = st->max_loops,
        .active_group_id = st->active_group_id,
    };

    uint16_t payload_len =
        proto_encode_motion_cycle_status_resp(&resp, payload, (uint16_t)sizeof(payload));
    if (payload_len == 0U) {
        return false;
    }
    return send_motion_payload(payload, payload_len);
}

static bool encode_and_send_cycle_list(void)
{
    proto_motion_cycle_status_t cycles[MAX_CYCLE];
    uint8_t cycle_count = 0U;

    for (uint8_t i = 0; i < MAX_CYCLE; ++i) {
        motion_cycle_status_t st;
        if (!motion_cycle_get_status(i, &st)) {
            continue;
        }

        cycles[cycle_count].index = i;
        cycles[cycle_count].active = (uint8_t)st.active;
        cycles[cycle_count].running = (uint8_t)st.running;
        cycles[cycle_count].current_pose = st.current_pose_index;
        cycles[cycle_count].pose_count = st.pose_count;
        cycles[cycle_count].loop_count = st.loop_count;
        cycles[cycle_count].max_loops = st.max_loops;
        cycles[cycle_count].active_group_id = st.active_group_id;
        cycle_count++;

        proto_cycle_data_t* pdata = (proto_cycle_data_t*)st.user_data;
        if (pdata && pdata->allocated) {
            MOTION_LOG(
                "cycle[%u]: protocol_cycle mode=%u active=%u running=%u pose=%u/%u loops=%lu/%lu group=%lu",
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
            MOTION_LOG("cycle[%u]: internal_cycle active=%u running=%u pose=%u/%u loops=%lu/%lu group=%lu",
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

    uint8_t payload[PROTO_MAX_PAYLOAD];
    proto_motion_cycle_list_resp_t resp = {
        .subcmd = (uint8_t)MOTION_CMD_CYCLE_LIST,
        .cycle_count = cycle_count,
        .cycles = cycles,
    };

    uint16_t payload_len =
        proto_encode_motion_cycle_list_resp(&resp, payload, (uint16_t)sizeof(payload));
    if (payload_len == 0U) {
        return false;
    }

    MOTION_LOG("CYCLE_LIST count=%u total_len=%u", (unsigned)cycle_count, (unsigned)payload_len);
    return send_motion_payload(payload, payload_len);
}

// Find free protocol cycle data slot
static int8_t find_free_proto_cycle_data(void)
{
    for (int i = 0; i < MAX_CYCLE; ++i) {
        if (s_proto_cycle_data[i].allocated == 0) {
            return (int8_t)i;
        }
    }
    return -1;
}

// Release protocol cycle data slot
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
    // Fetch protocol user data.
    void* user_data = motion_cycle_get_user_data(cycle_index);
    if (user_data == NULL) {
        // Not a protocol-created cycle; no status push.
        return;
    }

    // Validate allocation state.
    proto_cycle_data_t* pdata = (proto_cycle_data_t*)user_data;
    if (pdata->allocated == 0) {
        // Slot not allocated, may be an internal cycle.
        return;
    }

    // Check notify mask.
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
    (void)encode_and_send_cycle_status_update(cycle_index, loop_count, remaining, finished ? 1U : 0U);

    MOTION_LOG("Cycle status: index=%lu loop=%lu/%lu remaining=%lu finished=%u",
               (unsigned long)cycle_index,
               (unsigned long)loop_count,
               (unsigned long)max_loops,
               (unsigned long)remaining,
               (unsigned)finished);
}

static void protocol_motion_group_done(uint32_t group_id)
{
    (void)encode_and_send_motion_status((uint8_t)MOTION_CMD_STATUS, group_id, 1U);
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
            // Payload format: [mode:u8][count:u8][duration:u32][ids...][values...]
            proto_motion_start_req_t req;
            if (!proto_decode_motion_start(payload, len, &req)) {
                return false;
            }
            MOTION_LOG("mode=%u count=%u duration=%lu",
                       (unsigned)req.mode,
                       (unsigned)req.servo_count,
                       (unsigned long)req.duration_ms);
            if (req.servo_count == 0U || req.servo_count > MAX_SERVOS) {
                return false;
            }

            uint32_t gid = 0;
            if (req.mode == 0U) {
                uint32_t pwms[MAX_SERVOS];
                for (uint8_t i = 0; i < req.servo_count; ++i) {
                    memcpy(&pwms[i], &req.values_raw[(uint16_t)i * 4U], sizeof(uint32_t));
                    MOTION_LOG("ids[%u]=%u pwm[%u]=%lu",
                               (unsigned)i,
                               (unsigned)req.servo_ids[i],
                               (unsigned)i,
                               (unsigned long)pwms[i]);
                }
                gid = motion_sync_move_pwm(req.servo_ids,
                                           pwms,
                                           req.servo_count,
                                           req.duration_ms,
                                           protocol_motion_group_done);
            } else if (req.mode == 1U) {
                float angles[MAX_SERVOS];
                for (uint8_t i = 0; i < req.servo_count; ++i) {
                    memcpy(&angles[i], &req.values_raw[(uint16_t)i * 4U], sizeof(float));
                    MOTION_LOG("ids[%u]=%u angle[%u]=%.3f",
                               (unsigned)i,
                               (unsigned)req.servo_ids[i],
                               (unsigned)i,
                               (double)angles[i]);
                }
                gid = motion_sync_move_angle(req.servo_ids,
                                             angles,
                                             req.servo_count,
                                             req.duration_ms,
                                             protocol_motion_group_done);
            } else {
                return false;
            }

            return encode_and_send_motion_start(gid);
        }
        case MOTION_CMD_STOP: {
            MOTION_LOG("CMD STOP");
            MOTION_DUMP("payload", payload, len);
            uint32_t gid = 0;
            if (!proto_decode_motion_id(payload, len, &gid)) {
                return false;
            }
            MOTION_LOG("group_id=%lu", (unsigned long)gid);
            return motion_sync_release_group(gid);
        }
        case MOTION_CMD_PAUSE: {
            MOTION_LOG("CMD PAUSE");
            MOTION_DUMP("payload", payload, len);
            uint32_t gid = 0;
            if (!proto_decode_motion_id(payload, len, &gid)) {
                return false;
            }
            MOTION_LOG("group_id=%lu", (unsigned long)gid);
            return motion_sync_pause_group(gid);
        }
        case MOTION_CMD_RESUME: {
            MOTION_LOG("CMD RESUME");
            MOTION_DUMP("payload", payload, len);
            uint32_t gid = 0;
            if (!proto_decode_motion_id(payload, len, &gid)) {
                return false;
            }
            MOTION_LOG("group_id=%lu", (unsigned long)gid);
            return motion_sync_restart_group(gid);
        }
        case MOTION_CMD_GET_STATUS: {
            MOTION_LOG("CMD GET_STATUS");
            MOTION_DUMP("payload", payload, len);
            uint32_t gid = 0;
            if (!proto_decode_motion_id(payload, len, &gid)) {
                return false;
            }
            MOTION_LOG("group_id=%lu", (unsigned long)gid);
            return encode_and_send_motion_get_status(
                gid,
                motion_sync_get_group_mask(gid),
                (uint8_t)(motion_sync_is_group_complete(gid) ? 1U : 0U));
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
            // Payload format:
            // [mode:u8][servo_count:u8][pose_count:u8][max_loops:u32]
            // [durations:u32 * pose_count][ids:u8 * servo_count]
            // [values:pose_count * servo_count * 4]
            proto_motion_cycle_create_req_t req;
            if (!proto_decode_motion_cycle_create(payload, len, &req)) {
                return false;
            }
            MOTION_LOG("mode=%u servo_count=%u pose_count=%u max_loops=%lu",
                       (unsigned)req.mode,
                       (unsigned)req.servo_count,
                       (unsigned)req.pose_count,
                       (unsigned long)req.max_loops);
            if (req.servo_count == 0U || req.pose_count == 0U) {
                return false;
            }
            if (req.servo_count > PROTO_CYCLE_MAX_SERVO || req.pose_count > PROTO_CYCLE_MAX_POSE) {
                return false;
            }

            // Allocate a protocol cycle data slot.
            int8_t data_slot = find_free_proto_cycle_data();
            if (data_slot < 0) {
                MOTION_LOG("No free protocol cycle data slot available");
                return false;
            }

            proto_cycle_data_t* pdata = &s_proto_cycle_data[data_slot];
            pdata->allocated          = 1;
            pdata->mode               = req.mode;
            pdata->servo_count        = req.servo_count;
            pdata->pose_count         = req.pose_count;

            // Copy servo IDs.
            for (uint8_t i = 0; i < req.servo_count; ++i) {
                pdata->servo_ids[i] = req.servo_ids[i];
                MOTION_LOG("ids[%u]=%u", (unsigned)i, (unsigned)pdata->servo_ids[i]);
            }

            // Copy per-pose durations.
            for (uint8_t p = 0; p < req.pose_count; ++p) {
                uint32_t dur = 0U;
                memcpy(&dur, &req.pose_durations_raw[(uint16_t)p * 4U], sizeof(uint32_t));
                pdata->pose_duration[p] = dur;
                MOTION_LOG("durations[%u]=%lu", (unsigned)p, (unsigned long)dur);
            }

            // Build value pointers and copy pose values.
            for (uint8_t p = 0; p < req.pose_count; ++p) {
                if (req.mode == 0U) {
                    pdata->pose_pwm_ptrs[p] = pdata->pose_pwm[p];
                    for (uint8_t i = 0; i < req.servo_count; ++i) {
                        uint16_t value_index = (uint16_t)p * req.servo_count + i;
                        uint32_t pwm = 0U;
                        memcpy(&pwm, &req.values_raw[value_index * 4U], sizeof(uint32_t));
                        pdata->pose_pwm[p][i] = pwm;
                        MOTION_LOG(
                            "pose_pwm[%u][%u]=%lu", (unsigned)p, (unsigned)i, (unsigned long)pwm);
                    }
                } else if (req.mode == 1U) {
                    pdata->pose_angle_ptrs[p] = pdata->pose_angle[p];
                    for (uint8_t i = 0; i < req.servo_count; ++i) {
                        uint16_t value_index = (uint16_t)p * req.servo_count + i;
                        float angle = 0.0f;
                        memcpy(&angle, &req.values_raw[value_index * 4U], sizeof(float));
                        pdata->pose_angle[p][i] = angle;
                        MOTION_LOG(
                            "pose_angle[%u][%u]=%.3f", (unsigned)p, (unsigned)i, (double)angle);
                    }
                }
            }

            // Build motion_cycle config.
            motion_cycle_config_t config = {
                .servo_ids     = pdata->servo_ids,
                .servo_count   = req.servo_count,
                .pose_duration = pdata->pose_duration,
                .pose_count    = req.pose_count,
                .max_loops     = req.max_loops,
                .mode          = req.mode,
                .user_data     = pdata  // Keep protocol data pointer
            };

            if (req.mode == 0U) {
                config.pose_list_pwm = pdata->pose_pwm_ptrs;
            } else {
                config.pose_list_angle = pdata->pose_angle_ptrs;
            }

            // Create cycle.
            int32_t cycle_index = motion_cycle_create(&config, protocol_cycle_status_cb);
            if (cycle_index < 0) {
                MOTION_LOG("motion_cycle_create failed");
                release_proto_cycle_data((uint8_t)data_slot);
                return false;
            }

            // Set back-reference user data.
            if (!motion_cycle_set_user_data(cycle_index, pdata)) {
                MOTION_LOG("motion_cycle_set_user_data failed");
                motion_cycle_release(cycle_index);
                release_proto_cycle_data((uint8_t)data_slot);
                return false;
            }

            // Update notify mask.
            if (cycle_index < 32) {
                s_cycle_notify_mask |= (1U << cycle_index);
            }

            MOTION_LOG("CYCLE_CREATE success: index=%ld data_slot=%u",
                       (long)cycle_index,
                       (unsigned)data_slot);

            // Print all current cycle states.
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

            return encode_and_send_cycle_list();
        }
        case MOTION_CMD_CYCLE_START: {
            MOTION_LOG("CMD CYCLE_START");
            MOTION_DUMP("payload", payload, len);
            uint32_t idx = 0;
            if (!proto_decode_motion_id(payload, len, &idx)) {
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
            if (!proto_decode_motion_id(payload, len, &idx)) {
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
            if (!proto_decode_motion_id(payload, len, &idx)) {
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
            if (!proto_decode_motion_id(payload, len, &idx)) {
                return false;
            }

            // Fetch protocol user data.
            proto_cycle_data_t* pdata = (proto_cycle_data_t*)motion_cycle_get_user_data(idx);
            if (pdata && pdata->allocated) {
                // Compute data slot index.
                int8_t data_slot = (int8_t)(pdata - s_proto_cycle_data);
                if (data_slot >= 0 && data_slot < MAX_CYCLE) {
                    release_proto_cycle_data((uint8_t)data_slot);
                    MOTION_LOG("Released protocol cycle data slot=%d for cycle=%lu",
                               data_slot,
                               (unsigned long)idx);
                }
            }

            // Clear notify mask.
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
            if (!proto_decode_motion_id(payload, len, &idx)) {
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

            // Detect protocol-created cycles for debug logs.
            proto_cycle_data_t* pdata = (proto_cycle_data_t*)st.user_data;
            if (pdata && pdata->allocated) {
                MOTION_LOG("cycle[%lu] is protocol cycle, mode=%u",
                           (unsigned long)idx,
                           (unsigned)pdata->mode);
            }

            return encode_and_send_cycle_status(idx, &st);
        }
        case MOTION_CMD_CYCLE_LIST: {
            MOTION_LOG("CMD CYCLE_LIST");
            MOTION_DUMP("payload", payload, len);
            return encode_and_send_cycle_list();
        }
        default:
            return false;
    }
}
