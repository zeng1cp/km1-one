#include "protocol.h"
#include "tinyframe/TinyFrame.h"
#include "motion_engine.h"
#include "motion_sync.h"

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

    if (protocol_motion_handle(cmd_view.cmd, cmd_view.payload, cmd_view.payload_len)) {
        return TF_STAY;
    }

    return TF_NEXT;
}

bool protocol_motion_handle(uint8_t cmd, const uint8_t* payload, uint16_t len)
{
    // Payload format: [cmd][payload...]
    switch (cmd) {
        case MOTION_CMD_START: {
            // payload: [mode:u8][count:u8][duration:u32][ids...][values...]
            if (len < 6) {
                return false;
            }
            uint8_t mode  = payload[0];  // 0=pwm(u32), 1=angle(f32)
            uint8_t count = payload[1];
            uint32_t duration = 0;
            if (!proto_read_u32_le(payload, len, 2, &duration)) {
                return false;
            }
            uint16_t ids_off = 6;
            uint16_t values_off = (uint16_t)(ids_off + count);
            uint16_t value_size = 4;
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
                    if (!proto_read_u32_le(payload, len, (uint16_t)(values_off + i * 4), &pwms[i])) {
                        return false;
                    }
                }
                gid = motion_sync_move_pwm(&payload[ids_off], pwms, count, duration, NULL);
            } else if (mode == 1) {
                float angles[MAX_SERVOS];
                if (count > MAX_SERVOS) {
                    return false;
                }
                for (uint8_t i = 0; i < count; ++i) {
                    if (!proto_read_f32_le(payload, len, (uint16_t)(values_off + i * 4), &angles[i])) {
                        return false;
                    }
                }
                gid = motion_sync_move_angle(&payload[ids_off], angles, count, duration, NULL);
            } else {
                return false;
            }

            uint8_t resp[4];
            proto_write_u32_le(resp, 0, gid);
            protocol_send_state(STATE_CMD_MOTION, resp, (uint16_t)sizeof(resp));
            return true;
        }
        case MOTION_CMD_STOP: {
            uint32_t gid = 0;
            if (!proto_read_u32_le(payload, len, 0, &gid)) {
                return false;
            }
            return motion_sync_release_group(gid);
        }
        case MOTION_CMD_PAUSE: {
            uint32_t gid = 0;
            if (!proto_read_u32_le(payload, len, 0, &gid)) {
                return false;
            }
            return motion_sync_pause_group(gid);
        }
        case MOTION_CMD_RESUME: {
            uint32_t gid = 0;
            if (!proto_read_u32_le(payload, len, 0, &gid)) {
                return false;
            }
            return motion_sync_restart_group(gid);
        }
        case MOTION_CMD_GET_STATUS: {
            uint32_t gid = 0;
            if (!proto_read_u32_le(payload, len, 0, &gid)) {
                return false;
            }
            uint8_t resp[9];
            proto_write_u32_le(resp, 0, gid);
            proto_write_u32_le(resp, 4, motion_sync_get_group_mask(gid));
            resp[8] = (uint8_t)(motion_sync_is_group_complete(gid) ? 1 : 0);
            return protocol_send_state(STATE_CMD_MOTION, resp, (uint16_t)sizeof(resp));
        }
        case MOTION_CMD_SET_PLAN:
            return false;
        case MOTION_CMD_STATUS:
            return true;
        default:
            return false;
    }
}
