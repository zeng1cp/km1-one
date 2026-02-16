#include "motion_codec.h"

#include <string.h>

#include "protocol_codec.h"

// ------------------------------------------------------------------
// Decode functions: bounds checks only, no business validation
// ------------------------------------------------------------------
bool proto_decode_motion_start(const uint8_t* payload, uint16_t len, proto_motion_start_req_t* out)
{
    if (!payload || !out || len < 6) return false;  // minimum fixed header size
    out->mode        = payload[0];
    out->servo_count = payload[1];
    if (!proto_read_u32_le(payload, len, 2, &out->duration_ms)) return false;

    uint16_t ids_off      = 6;
    uint16_t values_off   = ids_off + out->servo_count;
    uint16_t total_needed = values_off + (uint16_t)out->servo_count * 4;
    if (total_needed > len) return false;

    out->servo_ids = &payload[ids_off];
    out->values_raw = &payload[values_off];
    if (out->mode == 0) {
        out->values.pwms = (const uint32_t*)&payload[values_off];
    } else if (out->mode == 1) {
        out->values.angles = (const float*)&payload[values_off];
    } else {
        return false;  // invalid mode
    }
    return true;
}

bool proto_decode_motion_id(const uint8_t* payload, uint16_t len, uint32_t* out_id)
{
    if (!payload || !out_id || len < 4) return false;
    return proto_read_u32_le(payload, len, 0, out_id);
}



// ------------------------------------------------------------------
// Encode functions: buffer size checks only
// ------------------------------------------------------------------
uint16_t proto_encode_motion_start_resp(const proto_motion_start_resp_t* resp,
                                        uint8_t*                         buf,
                                        uint16_t                         buf_size)
{
    if (!resp || !buf || buf_size < 5) return 0;
    buf[0] = resp->subcmd;
    proto_write_u32_le(buf, 1, resp->group_id);
    return 5;
}

uint16_t proto_encode_motion_status_resp(const proto_motion_status_resp_t* resp,
                                         uint8_t*                          buf,
                                         uint16_t                          buf_size)
{
    if (!resp || !buf || buf_size < 6) return 0;
    buf[0] = resp->subcmd;
    proto_write_u32_le(buf, 1, resp->group_id);
    buf[5] = resp->complete;
    return 6;
}

uint16_t proto_encode_motion_get_status_resp(const proto_motion_get_status_resp_t* resp,
                                             uint8_t*                              buf,
                                             uint16_t                              buf_size)
{
    if (!resp || !buf || buf_size < 10) return 0;
    buf[0] = resp->subcmd;
    proto_write_u32_le(buf, 1, resp->group_id);
    proto_write_u32_le(buf, 5, resp->mask);
    buf[9] = resp->complete;
    return 10;
}

