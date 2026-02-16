#include "cycle_codec.h"

#include "protocol_codec.h"

bool proto_decode_cycle_id(const uint8_t* payload, uint16_t len, uint32_t* out_id)
{
    if (!payload || !out_id || len < 4) return false;
    return proto_read_u32_le(payload, len, 0, out_id);
}

bool proto_decode_cycle_create(const uint8_t* payload, uint16_t len, proto_cycle_create_req_t* out)
{
    if (!payload || !out || len < 7) return false;  // minimum fixed header size
    out->mode        = payload[0];
    out->servo_count = payload[1];
    out->pose_count  = payload[2];
    if (!proto_read_u32_le(payload, len, 3, &out->max_loops)) return false;

    uint16_t durations_off = 7;
    uint16_t ids_off       = durations_off + (uint16_t)out->pose_count * 4;
    uint16_t values_off    = ids_off + out->servo_count;
    uint16_t values_len    = (uint16_t)out->pose_count * (uint16_t)out->servo_count * 4;
    uint16_t total_needed  = values_off + values_len;
    if (total_needed > len) return false;

    out->pose_durations_raw = &payload[durations_off];
    out->pose_durations     = (const uint32_t*)&payload[durations_off];
    out->servo_ids          = &payload[ids_off];
    out->values_raw         = &payload[values_off];
    if (out->mode == 0) {
        out->values.pwms_flat = (const uint32_t*)&payload[values_off];
    } else if (out->mode == 1) {
        out->values.angles_flat = (const float*)&payload[values_off];
    } else {
        return false;
    }
    return true;
}

uint16_t proto_encode_cycle_list_resp(const proto_cycle_list_resp_t* resp,
                                      uint8_t*                       buf,
                                      uint16_t                       buf_size)
{
    if (!resp || !buf || buf_size < 2) return 0;
    buf[0]       = resp->subcmd;
    buf[1]       = resp->cycle_count;
    uint16_t pos = 2;
    for (uint8_t i = 0; i < resp->cycle_count; ++i) {
        const proto_cycle_status_t* st = &resp->cycles[i];
        if (pos + 17 > buf_size) return 0;
        buf[pos + 0] = st->index;
        buf[pos + 1] = st->active;
        buf[pos + 2] = st->running;
        buf[pos + 3] = st->current_pose;
        buf[pos + 4] = st->pose_count;
        proto_write_u32_le(buf, pos + 5, st->loop_count);
        proto_write_u32_le(buf, pos + 9, st->max_loops);
        proto_write_u32_le(buf, pos + 13, st->active_group_id);
        pos += 17;
    }
    return pos;
}

uint16_t proto_encode_cycle_status_resp(const proto_cycle_status_resp_t* resp,
                                        uint8_t*                         buf,
                                        uint16_t                         buf_size)
{
    if (!resp || !buf || buf_size < 21) return 0;
    buf[0] = resp->subcmd;
    proto_write_u32_le(buf, 1, resp->cycle_index);
    buf[5] = resp->active;
    buf[6] = resp->running;
    buf[7] = resp->current_pose;
    buf[8] = resp->pose_count;
    proto_write_u32_le(buf, 9, resp->loop_count);
    proto_write_u32_le(buf, 13, resp->max_loops);
    proto_write_u32_le(buf, 17, resp->active_group_id);
    return 21;
}

uint16_t proto_encode_cycle_status_update_resp(const proto_cycle_status_update_resp_t* resp,
                                               uint8_t*                                buf,
                                               uint16_t                                buf_size)
{
    if (!resp || !buf || buf_size < 14) return 0;
    buf[0] = resp->subcmd;
    proto_write_u32_le(buf, 1, resp->cycle_index);
    proto_write_u32_le(buf, 5, resp->loop_count);
    proto_write_u32_le(buf, 9, resp->remaining);
    buf[13] = resp->finished;
    return 14;
}
