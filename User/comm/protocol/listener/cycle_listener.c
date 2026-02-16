#include <stdio.h>
#include <string.h>

#include "cycle_codec.h"
#include "motion_cycle.h"
#include "motion_engine.h"
#include "protocol.h"
#include "tinyframe/TinyFrame.h"

#ifndef CYCLE_LISTENER_LOG_ENABLE
#define CYCLE_LISTENER_LOG_ENABLE 1
#endif

#if CYCLE_LISTENER_LOG_ENABLE
#define CYCLE_LOG(fmt, ...) printf("[cycle] " fmt "\n", ##__VA_ARGS__)
#define CYCLE_DUMP(label, data, len)                            \
    do {                                                        \
        printf("[cycle] %s (len=%u):", label, (unsigned)(len)); \
        for (uint16_t _i = 0; _i < (len); ++_i) {               \
            printf(" %02X", (unsigned)(data)[_i]);              \
        }                                                       \
        printf("\n");                                           \
    } while (0)
#else
#define CYCLE_LOG(fmt, ...)          ((void)0)
#define CYCLE_DUMP(label, data, len) ((void)0)
#endif

static bool send_cycle_payload(const uint8_t* payload, uint16_t payload_len)
{
    return protocol_send_state(STATE_CMD_CYCLE, payload, payload_len);
}

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

static bool encode_and_send_cycle_status_update(uint32_t cycle_index,
                                                uint32_t loop_count,
                                                uint32_t remaining,
                                                uint8_t  finished)
{
    uint8_t                          payload[14];
    proto_cycle_status_update_resp_t resp = {
        .subcmd      = (uint8_t)CYCLE_CMD_STATUS,
        .cycle_index = cycle_index,
        .loop_count  = loop_count,
        .remaining   = remaining,
        .finished    = finished,
    };

    uint16_t payload_len =
        proto_encode_cycle_status_update_resp(&resp, payload, (uint16_t)sizeof(payload));
    if (payload_len == 0U) {
        return false;
    }
    return send_cycle_payload(payload, payload_len);
}

static bool encode_and_send_cycle_status(uint32_t cycle_index, const motion_cycle_status_t* st)
{
    uint8_t payload[21];

    proto_cycle_status_resp_t resp = {
        .subcmd          = (uint8_t)CYCLE_CMD_GET_STATUS,
        .cycle_index     = cycle_index,
        .active          = (uint8_t)st->active,
        .running         = (uint8_t)st->running,
        .current_pose    = st->current_pose_index,
        .pose_count      = st->pose_count,
        .loop_count      = st->loop_count,
        .max_loops       = st->max_loops,
        .active_group_id = st->active_group_id,
    };

    uint16_t payload_len =
        proto_encode_cycle_status_resp(&resp, payload, (uint16_t)sizeof(payload));
    if (payload_len == 0U) {
        return false;
    }
    return send_cycle_payload(payload, payload_len);
}

static bool encode_and_send_cycle_list(void)
{
    proto_cycle_status_t cycles[MAX_CYCLE];
    uint8_t              cycle_count = 0U;

    for (uint8_t i = 0; i < MAX_CYCLE; ++i) {
        motion_cycle_status_t st;
        if (!motion_cycle_get_status(i, &st)) {
            continue;
        }

        cycles[cycle_count].index           = i;
        cycles[cycle_count].active          = (uint8_t)st.active;
        cycles[cycle_count].running         = (uint8_t)st.running;
        cycles[cycle_count].current_pose    = st.current_pose_index;
        cycles[cycle_count].pose_count      = st.pose_count;
        cycles[cycle_count].loop_count      = st.loop_count;
        cycles[cycle_count].max_loops       = st.max_loops;
        cycles[cycle_count].active_group_id = st.active_group_id;
        cycle_count++;

        proto_cycle_data_t* pdata = (proto_cycle_data_t*)st.user_data;
        if (pdata && pdata->allocated) {
            CYCLE_LOG(
                "cycle[%u]: protocol_cycle mode=%u active=%u running=%u pose=%u/%u loops=%lu/%lu "
                "group=%lu",
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
            CYCLE_LOG(
                "cycle[%u]: internal_cycle active=%u running=%u pose=%u/%u loops=%lu/%lu group=%lu",
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

    proto_cycle_list_resp_t resp = {
        .subcmd      = (uint8_t)CYCLE_CMD_LIST,
        .cycle_count = cycle_count,
        .cycles      = cycles,
    };

    uint16_t payload_len = proto_encode_cycle_list_resp(&resp, payload, (uint16_t)sizeof(payload));
    if (payload_len == 0U) {
        return false;
    }

    CYCLE_LOG("CYCLE_LIST count=%u total_len=%u", (unsigned)cycle_count, (unsigned)payload_len);
    return send_cycle_payload(payload, payload_len);
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

    uint32_t remaining =
        (max_loops == 0) ? 0xFFFFFFFFu : (max_loops > loop_count ? (max_loops - loop_count) : 0);
    (void)encode_and_send_cycle_status_update(
        cycle_index, loop_count, remaining, finished ? 1U : 0U);

    CYCLE_LOG("Cycle status: index=%lu loop=%lu/%lu remaining=%lu finished=%u",
              (unsigned long)cycle_index,
              (unsigned long)loop_count,
              (unsigned long)max_loops,
              (unsigned long)remaining,
              (unsigned)finished);

    motion_cycle_status_t st;
    if (!motion_cycle_get_status(cycle_index, &st)) {
        return;
    }

    (void)encode_and_send_cycle_status(cycle_index, &st);
}

TF_Result protocol_cycle_listener(TinyFrame* tf, TF_Msg* msg)
{
    (void)tf;
    if (msg == NULL) {
        return TF_NEXT;
    }

    proto_cmd_view_t cmd_view;
    if (!proto_parse_cmd(msg->data, msg->len, &cmd_view)) {
        return TF_NEXT;
    }

    (void)protocol_cycle_handle(cmd_view.cmd, cmd_view.payload, cmd_view.payload_len);
    return TF_STAY;
}

bool protocol_cycle_handle(uint8_t cmd, const uint8_t* payload, uint16_t len)
{
    // Payload format: [cmd][payload...]
    switch (cmd) {
        case CYCLE_CMD_CREATE: {
            CYCLE_LOG("CMD CYCLE_CREATE");
            CYCLE_DUMP("payload", payload, len);
            // Payload format:
            // [mode:u8][servo_count:u8][pose_count:u8][max_loops:u32]
            // [durations:u32 * pose_count][ids:u8 * servo_count]
            // [values:pose_count * servo_count * 4]
            proto_cycle_create_req_t req;
            if (!proto_decode_cycle_create(payload, len, &req)) {
                return false;
            }
            CYCLE_LOG("mode=%u servo_count=%u pose_count=%u max_loops=%lu",
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
                CYCLE_LOG("No free protocol cycle data slot available");
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
                CYCLE_LOG("ids[%u]=%u", (unsigned)i, (unsigned)pdata->servo_ids[i]);
            }

            // Copy per-pose durations.
            for (uint8_t p = 0; p < req.pose_count; ++p) {
                uint32_t dur = 0U;
                memcpy(&dur, &req.pose_durations_raw[(uint16_t)p * 4U], sizeof(uint32_t));
                pdata->pose_duration[p] = dur;
                CYCLE_LOG("durations[%u]=%lu", (unsigned)p, (unsigned long)dur);
            }

            // Build value pointers and copy pose values.
            for (uint8_t p = 0; p < req.pose_count; ++p) {
                if (req.mode == 0U) {
                    pdata->pose_pwm_ptrs[p] = pdata->pose_pwm[p];
                    for (uint8_t i = 0; i < req.servo_count; ++i) {
                        uint16_t value_index = (uint16_t)p * req.servo_count + i;
                        uint32_t pwm         = 0U;
                        memcpy(&pwm, &req.values_raw[value_index * 4U], sizeof(uint32_t));
                        pdata->pose_pwm[p][i] = pwm;
                        CYCLE_LOG(
                            "pose_pwm[%u][%u]=%lu", (unsigned)p, (unsigned)i, (unsigned long)pwm);
                    }
                } else if (req.mode == 1U) {
                    pdata->pose_angle_ptrs[p] = pdata->pose_angle[p];
                    for (uint8_t i = 0; i < req.servo_count; ++i) {
                        uint16_t value_index = (uint16_t)p * req.servo_count + i;
                        float    angle       = 0.0f;
                        memcpy(&angle, &req.values_raw[value_index * 4U], sizeof(float));
                        pdata->pose_angle[p][i] = angle;
                        CYCLE_LOG(
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
                CYCLE_LOG("motion_cycle_create failed");
                release_proto_cycle_data((uint8_t)data_slot);
                return false;
            }

            // Set back-reference user data.
            if (!motion_cycle_set_user_data(cycle_index, pdata)) {
                CYCLE_LOG("motion_cycle_set_user_data failed");
                motion_cycle_release(cycle_index);
                release_proto_cycle_data((uint8_t)data_slot);
                return false;
            }

            // Update notify mask.
            if (cycle_index < 32) {
                s_cycle_notify_mask |= (1U << cycle_index);
            }

            CYCLE_LOG("CYCLE_CREATE success: index=%ld data_slot=%u",
                      (long)cycle_index,
                      (unsigned)data_slot);

            // Print all current cycle states.
            for (uint8_t i = 0; i < MAX_CYCLE; ++i) {
                motion_cycle_status_t st;
                if (motion_cycle_get_status(i, &st)) {
                    CYCLE_LOG("cycle[%u]: active=%u running=%u pose=%u/%u loops=%lu/%lu group=%lu",
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
        case CYCLE_CMD_START: {
            CYCLE_LOG("CMD CYCLE_START");
            CYCLE_DUMP("payload", payload, len);
            uint32_t idx = 0;
            if (!proto_decode_cycle_id(payload, len, &idx)) {
                return false;
            }
            int result = motion_cycle_start(idx);
            if (result == 0) {
                CYCLE_LOG("CYCLE_START success: index=%lu", (unsigned long)idx);
                motion_cycle_status_t st;
                if (motion_cycle_get_status(idx, &st)) {
                    CYCLE_LOG("cycle[%lu]: active=%u running=%u pose=%u/%u loops=%lu/%lu group=%lu",
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
        case CYCLE_CMD_RESTART: {
            CYCLE_LOG("CMD CYCLE_RESTART");
            CYCLE_DUMP("payload", payload, len);
            uint32_t idx = 0;
            if (!proto_decode_cycle_id(payload, len, &idx)) {
                return false;
            }
            int result = motion_cycle_restart(idx);
            if (result == 0) {
                CYCLE_LOG("CYCLE_RESTART success: index=%lu", (unsigned long)idx);
                motion_cycle_status_t st;
                if (motion_cycle_get_status(idx, &st)) {
                    CYCLE_LOG("cycle[%lu]: active=%u running=%u pose=%u/%u loops=%lu/%lu group=%lu",
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
        case CYCLE_CMD_PAUSE: {
            CYCLE_LOG("CMD CYCLE_PAUSE");
            CYCLE_DUMP("payload", payload, len);
            uint32_t idx = 0;
            if (!proto_decode_cycle_id(payload, len, &idx)) {
                return false;
            }
            int result = motion_cycle_pause(idx);
            if (result == 0) {
                CYCLE_LOG("CYCLE_PAUSE success: index=%lu", (unsigned long)idx);
                motion_cycle_status_t st;
                if (motion_cycle_get_status(idx, &st)) {
                    CYCLE_LOG("cycle[%lu]: active=%u running=%u pose=%u/%u loops=%lu/%lu group=%lu",
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
        case CYCLE_CMD_RELEASE: {
            CYCLE_LOG("CMD CYCLE_RELEASE");
            CYCLE_DUMP("payload", payload, len);
            uint32_t idx = 0;
            if (!proto_decode_cycle_id(payload, len, &idx)) {
                return false;
            }

            // Fetch protocol user data.
            proto_cycle_data_t* pdata = (proto_cycle_data_t*)motion_cycle_get_user_data(idx);
            if (pdata && pdata->allocated) {
                // Compute data slot index.
                int8_t data_slot = (int8_t)(pdata - s_proto_cycle_data);
                if (data_slot >= 0 && data_slot < MAX_CYCLE) {
                    release_proto_cycle_data((uint8_t)data_slot);
                    CYCLE_LOG("Released protocol cycle data slot=%d for cycle=%lu",
                              data_slot,
                              (unsigned long)idx);
                }
            }

            int result = motion_cycle_release(idx);
            if (result == 0) {
                CYCLE_LOG("CYCLE_RELEASE success: index=%lu", (unsigned long)idx);
            }

            // Clear notify mask.
            if (idx < 32) {
                s_cycle_notify_mask &= ~(1U << idx);
            }

            return encode_and_send_cycle_list();
        }
        case CYCLE_CMD_GET_STATUS:
        case CYCLE_CMD_STATUS: {
            CYCLE_LOG("CMD CYCLE_GET_STATUS/STATUS");
            CYCLE_DUMP("payload", payload, len);
            uint32_t idx = 0;
            if (!proto_decode_cycle_id(payload, len, &idx)) {
                return false;
            }
            CYCLE_LOG("cycle_index=%lu", (unsigned long)idx);
            motion_cycle_status_t st;
            if (!motion_cycle_get_status(idx, &st)) {
                return false;
            }
            CYCLE_LOG(
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
                CYCLE_LOG("cycle[%lu] is protocol cycle, mode=%u",
                          (unsigned long)idx,
                          (unsigned)pdata->mode);
            }

            return encode_and_send_cycle_status(idx, &st);
        }
        case CYCLE_CMD_LIST: {
            CYCLE_LOG("CMD CYCLE_LIST");
            CYCLE_DUMP("payload", payload, len);
            return encode_and_send_cycle_list();
        }
        default:
            return false;
    }
}