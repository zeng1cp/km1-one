#ifndef ARM_CODEC_H
#define ARM_CODEC_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t moving_mask;
} proto_arm_status_resp_t;

typedef struct {
    uint32_t duration_ms;
} proto_arm_home_req_t;

typedef struct {
    uint32_t duration_ms;
    const uint8_t* angles_raw;
} proto_arm_set_pose_req_t;

bool proto_decode_arm_home_req(const uint8_t* payload, uint16_t len, uint32_t default_duration_ms, proto_arm_home_req_t* out);
bool proto_decode_arm_set_pose_req(const uint8_t* payload, uint16_t len, uint8_t joint_count, proto_arm_set_pose_req_t* out);

uint16_t proto_encode_arm_status_resp(const proto_arm_status_resp_t* resp,
                                      uint8_t* buf,
                                      uint16_t buf_size);

#ifdef __cplusplus
}
#endif

#endif  // ARM_CODEC_H
