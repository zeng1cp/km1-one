#ifndef PROTOCOL_CODEC_H
#define PROTOCOL_CODEC_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t cmd;
    const uint8_t* payload;
    uint16_t payload_len;
} proto_cmd_view_t;

bool proto_parse_cmd(const uint8_t* data, uint16_t len, proto_cmd_view_t* out);

bool proto_read_u16_le(const uint8_t* data, uint16_t len, uint16_t off, uint16_t* out);
bool proto_read_u32_le(const uint8_t* data, uint16_t len, uint16_t off, uint32_t* out);
bool proto_read_f32_le(const uint8_t* data, uint16_t len, uint16_t off, float* out);

void proto_write_u32_le(uint8_t* data, uint16_t off, uint32_t value);
void proto_write_f32_le(uint8_t* data, uint16_t off, float value);

bool proto_encode_cmd_frame(uint8_t cmd,
                            const uint8_t* payload,
                            uint16_t payload_len,
                            uint8_t* out,
                            uint16_t out_size,
                            uint16_t* out_len);

#ifdef __cplusplus
}
#endif

#endif  // PROTOCOL_CODEC_H
