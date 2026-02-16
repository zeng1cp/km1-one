#include <stdio.h>
#include <string.h>

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
        
        default:
            return false;
    }
}
