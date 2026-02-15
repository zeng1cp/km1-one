#ifndef SERVO_CODEC_H
#define SERVO_CODEC_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t subcmd;
    uint32_t servo_id;
    uint8_t moving;
    uint32_t current_pwm;
    float target_angle;
    uint32_t remaining_time;
} proto_servo_status_resp_t;

typedef struct {
    uint8_t id;
    uint32_t pwm;
    uint32_t duration_ms;
} proto_servo_set_pwm_req_t;

typedef struct {
    uint8_t id;
    float angle;
    uint32_t duration_ms;
} proto_servo_set_pos_req_t;

typedef struct {
    uint32_t duration_ms;
} proto_servo_home_req_t;

bool proto_decode_servo_id_req(const uint8_t* payload, uint16_t len, uint8_t* out_id);
bool proto_decode_servo_set_pwm_req(const uint8_t* payload, uint16_t len, proto_servo_set_pwm_req_t* out);
bool proto_decode_servo_set_pos_req(const uint8_t* payload, uint16_t len, proto_servo_set_pos_req_t* out);
bool proto_decode_servo_home_req(const uint8_t* payload, uint16_t len, proto_servo_home_req_t* out);

uint16_t proto_encode_servo_status_resp(const proto_servo_status_resp_t* resp,
                                        uint8_t* buf,
                                        uint16_t buf_size);

#ifdef __cplusplus
}
#endif

#endif  // SERVO_CODEC_H
