#include "arm_codec.h"

#include "protocol_codec.h"

bool proto_decode_arm_home_req(const uint8_t* payload, uint16_t len, uint32_t default_duration_ms, proto_arm_home_req_t* out)
{
    if (out == 0) {
        return false;
    }
    out->duration_ms = default_duration_ms;

    if (len == 0U) {
        return true;
    }
    if (payload == 0 || len != 4U) {
        return false;
    }
    return proto_read_u32_le(payload, len, 0U, &out->duration_ms);
}

bool proto_decode_arm_set_pose_req(const uint8_t* payload, uint16_t len, uint8_t joint_count, proto_arm_set_pose_req_t* out)
{
    uint16_t expected = (uint16_t)(4U + (uint16_t)joint_count * 4U);
    if (payload == 0 || out == 0 || len != expected) {
        return false;
    }
    if (!proto_read_u32_le(payload, len, 0U, &out->duration_ms)) {
        return false;
    }
    out->angles_raw = &payload[4];
    return true;
}

uint16_t proto_encode_arm_status_resp(const proto_arm_status_resp_t* resp,
                                      uint8_t* buf,
                                      uint16_t buf_size)
{
    if (resp == 0 || buf == 0 || buf_size < 4U) {
        return 0U;
    }

    proto_write_u32_le(buf, 0U, resp->moving_mask);
    return 4U;
}
