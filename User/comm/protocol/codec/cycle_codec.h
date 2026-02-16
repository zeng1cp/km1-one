#ifndef CYCLE_CODEC_H
#define CYCLE_CODEC_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ------------------------------------------------------------------
// Codec-owned response status model (independent from business types)
// ------------------------------------------------------------------
typedef struct {
    uint8_t  index;
    uint8_t  active;
    uint8_t  running;
    uint8_t  current_pose;
    uint8_t  pose_count;
    uint32_t loop_count;
    uint32_t max_loops;
    uint32_t active_group_id;
} proto_cycle_status_t;

// ------------------------------------------------------------------
// Request models (decode output)
// ------------------------------------------------------------------
typedef struct {
    uint8_t  mode;
    uint8_t  servo_count;
    uint8_t  pose_count;
    uint32_t max_loops;
    const uint8_t*  pose_durations_raw;
    const uint32_t* pose_durations;
    const uint8_t*  servo_ids;
    const uint8_t*  values_raw;
    union {
        const uint32_t* pwms_flat;
        const float*    angles_flat;
    } values;
} proto_cycle_create_req_t;

// ------------------------------------------------------------------
// Response models (encode input)
// ------------------------------------------------------------------
typedef struct {
    uint8_t  subcmd;                       // CYCLE_CMD_LIST
    uint8_t  cycle_count;
    const proto_cycle_status_t* cycles;
} proto_cycle_list_resp_t;

typedef struct {
    uint8_t  subcmd;                       // CYCLE_CMD_GET_STATUS
    uint32_t cycle_index;
    uint8_t  active;
    uint8_t  running;
    uint8_t  current_pose;
    uint8_t  pose_count;
    uint32_t loop_count;
    uint32_t max_loops;
    uint32_t active_group_id;
} proto_cycle_status_resp_t;

typedef struct {
    uint8_t  subcmd;                       // CYCLE_CMD_STATUS
    uint32_t cycle_index;
    uint32_t loop_count;
    uint32_t remaining;
    uint8_t  finished;
} proto_cycle_status_update_resp_t;

// ------------------------------------------------------------------
// Decode API
// ------------------------------------------------------------------
bool proto_decode_cycle_create(const uint8_t* payload, uint16_t len,
                               proto_cycle_create_req_t* out);
bool proto_decode_cycle_id(const uint8_t* payload, uint16_t len,
                           uint32_t* out_id);

// ------------------------------------------------------------------
// Encode API
// ------------------------------------------------------------------
uint16_t proto_encode_cycle_list_resp(const proto_cycle_list_resp_t* resp,
                                      uint8_t* buf, uint16_t buf_size);
uint16_t proto_encode_cycle_status_resp(const proto_cycle_status_resp_t* resp,
                                        uint8_t* buf, uint16_t buf_size);
uint16_t proto_encode_cycle_status_update_resp(const proto_cycle_status_update_resp_t* resp,
                                               uint8_t* buf, uint16_t buf_size);

#ifdef __cplusplus
}
#endif

#endif /*CYCLE_CODEC_H*/
