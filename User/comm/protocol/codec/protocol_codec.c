#include "protocol_codec.h"

#include <string.h>

bool proto_parse_cmd(const uint8_t* data, uint16_t len, proto_cmd_view_t* out)
{
    if (out == NULL) {
        return false;
    }
    if (data == NULL || len < 1U) {
        out->cmd         = 0U;
        out->payload     = NULL;
        out->payload_len = 0U;
        return false;
    }
    out->cmd         = data[0];
    out->payload     = &data[1];
    out->payload_len = (uint16_t)(len - 1U);
    return true;
}

bool proto_read_u16_le(const uint8_t* data, uint16_t len, uint16_t off, uint16_t* out)
{
    if (data == NULL || out == NULL) {
        return false;
    }
    if ((uint32_t)off + 2U > len) {
        return false;
    }
    *out = (uint16_t)data[off] | (uint16_t)((uint16_t)data[off + 1U] << 8U);
    return true;
}

bool proto_read_u32_le(const uint8_t* data, uint16_t len, uint16_t off, uint32_t* out)
{
    if (data == NULL || out == NULL) {
        return false;
    }
    if ((uint32_t)off + 4U > len) {
        return false;
    }
    *out = (uint32_t)data[off] | ((uint32_t)data[off + 1U] << 8U) | ((uint32_t)data[off + 2U] << 16U)
         | ((uint32_t)data[off + 3U] << 24U);
    return true;
}

bool proto_read_f32_le(const uint8_t* data, uint16_t len, uint16_t off, float* out)
{
    uint32_t raw = 0U;
    if (out == NULL) {
        return false;
    }
    if (!proto_read_u32_le(data, len, off, &raw)) {
        return false;
    }
    memcpy(out, &raw, sizeof(float));
    return true;
}

void proto_write_u32_le(uint8_t* data, uint16_t off, uint32_t value)
{
    data[off + 0U] = (uint8_t)(value & 0xFFU);
    data[off + 1U] = (uint8_t)((value >> 8U) & 0xFFU);
    data[off + 2U] = (uint8_t)((value >> 16U) & 0xFFU);
    data[off + 3U] = (uint8_t)((value >> 24U) & 0xFFU);
}

void proto_write_f32_le(uint8_t* data, uint16_t off, float value)
{
    uint32_t raw = 0U;
    memcpy(&raw, &value, sizeof(float));
    proto_write_u32_le(data, off, raw);
}

bool proto_encode_cmd_frame(uint8_t cmd,
                            const uint8_t* payload,
                            uint16_t payload_len,
                            uint8_t* out,
                            uint16_t out_size,
                            uint16_t* out_len)
{
    if (out_len == NULL) {
        return false;
    }
    *out_len = 0U;

    if (out == NULL) {
        return false;
    }
    if ((uint32_t)payload_len + 1U > out_size) {
        return false;
    }
    if (payload_len > 0U && payload == NULL) {
        return false;
    }

    out[0] = cmd;
    if (payload_len > 0U) {
        memcpy(&out[1], payload, payload_len);
    }

    *out_len = (uint16_t)(payload_len + 1U);
    return true;
}
