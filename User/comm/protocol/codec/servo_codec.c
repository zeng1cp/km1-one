#include "servo_codec.h"

#include "protocol_codec.h"

bool proto_decode_servo_id_req(const uint8_t* payload, uint16_t len, uint8_t* out_id)
{
    if (payload == 0 || out_id == 0 || len != 1U) {
        return false;
    }
    *out_id = payload[0];
    return true;
}

bool proto_decode_servo_set_pwm_req(const uint8_t* payload, uint16_t len, proto_servo_set_pwm_req_t* out)
{
    if (payload == 0 || out == 0 || len != 9U) {
        return false;
    }
    out->id = payload[0];
    if (!proto_read_u32_le(payload, len, 1U, &out->pwm)) {
        return false;
    }
    return proto_read_u32_le(payload, len, 5U, &out->duration_ms);
}

bool proto_decode_servo_set_pos_req(const uint8_t* payload, uint16_t len, proto_servo_set_pos_req_t* out)
{
    if (payload == 0 || out == 0 || len != 9U) {
        return false;
    }
    out->id = payload[0];
    if (!proto_read_f32_le(payload, len, 1U, &out->angle)) {
        return false;
    }
    return proto_read_u32_le(payload, len, 5U, &out->duration_ms);
}

bool proto_decode_servo_home_req(const uint8_t* payload,
                                 uint16_t len,
                                 proto_servo_home_req_t* out)
{
    if (out == 0) {
        return false;
    }

    // One-click home: no payload, fixed duration 1000ms.
    (void)payload;
    if (len != 0U) {
        return false;
    }
    out->duration_ms = 1000U;
    return true;
}

uint16_t proto_encode_servo_status_resp(const proto_servo_status_resp_t* resp,
                                        uint8_t* buf,
                                        uint16_t buf_size)
{
    if (resp == 0 || buf == 0 || buf_size < 18U) {
        return 0U;
    }

    buf[0] = resp->subcmd;
    proto_write_u32_le(buf, 1U, resp->servo_id);
    buf[5] = resp->moving;
    proto_write_u32_le(buf, 6U, resp->current_pwm);
    proto_write_f32_le(buf, 10U, resp->target_angle);
    proto_write_u32_le(buf, 14U, resp->remaining_time);
    return 18U;
}
