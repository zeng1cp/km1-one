#include "protocol.h"
#include "tinyframe/TinyFrame.h"

#include "motion_cycle.h"
#include "motion_engine.h"

#define PROTO_CYCLE_MAX_SERVO MAX_SERVOS
#define PROTO_CYCLE_MAX_POSE  8
#define PROTO_CYCLE_MAX_SLOT  5

static uint8_t  s_cycle_used[PROTO_CYCLE_MAX_SLOT];
static uint8_t  s_cycle_slot_of_index[PROTO_CYCLE_MAX_SLOT];

static uint8_t  s_cycle_servo_ids[PROTO_CYCLE_MAX_SLOT][PROTO_CYCLE_MAX_SERVO];
static uint32_t s_cycle_pose_pwm[PROTO_CYCLE_MAX_SLOT][PROTO_CYCLE_MAX_POSE][PROTO_CYCLE_MAX_SERVO];
static float    s_cycle_pose_angle[PROTO_CYCLE_MAX_SLOT][PROTO_CYCLE_MAX_POSE][PROTO_CYCLE_MAX_SERVO];
static uint32_t s_cycle_durations[PROTO_CYCLE_MAX_SLOT][PROTO_CYCLE_MAX_POSE];

static uint32_t* s_cycle_pose_pwm_ptrs[PROTO_CYCLE_MAX_SLOT][PROTO_CYCLE_MAX_POSE];
static float*    s_cycle_pose_angle_ptrs[PROTO_CYCLE_MAX_SLOT][PROTO_CYCLE_MAX_POSE];

static int8_t find_free_cycle_slot(void)
{
    for (int i = 0; i < PROTO_CYCLE_MAX_SLOT; ++i) {
        if (s_cycle_used[i] == 0) {
            return (int8_t)i;
        }
    }
    return -1;
}

static void release_cycle_slot(uint8_t slot)
{
    if (slot >= PROTO_CYCLE_MAX_SLOT) {
        return;
    }
    s_cycle_used[slot] = 0;
}

TF_Result protocol_motion_cycle_listener(TinyFrame* tf, TF_Msg* msg)
{
    (void)tf;
    if (msg == NULL) {
        return TF_NEXT;
    }

    proto_cmd_view_t cmd_view;
    if (!proto_parse_cmd(msg->data, msg->len, &cmd_view)) {
        return TF_NEXT;
    }

    if (protocol_motion_cycle_handle(cmd_view.cmd, cmd_view.payload, cmd_view.payload_len)) {
        return TF_STAY;
    }

    return TF_NEXT;
}

bool protocol_motion_cycle_handle(uint8_t cmd, const uint8_t* payload, uint16_t len)
{
    switch (cmd) {
        case CYCLE_CMD_CREATE: {
            // payload: [mode:u8][servo_count:u8][pose_count:u8][max_loops:u32]
            //          [durations:u32 * pose_count][ids:u8 * servo_count]
            //          [values:pose_count * servo_count * 4]
            if (len < 7) {
                return false;
            }
            uint8_t mode = payload[0];  // 0=pwm(u32), 1=angle(f32)
            uint8_t servo_count = payload[1];
            uint8_t pose_count = payload[2];
            uint32_t max_loops = 0;
            if (!proto_read_u32_le(payload, len, 3, &max_loops)) {
                return false;
            }
            if (servo_count == 0 || pose_count == 0) {
                return false;
            }
            if (servo_count > PROTO_CYCLE_MAX_SERVO || pose_count > PROTO_CYCLE_MAX_POSE) {
                return false;
            }

            uint16_t durations_off = 7;
            uint16_t ids_off = (uint16_t)(durations_off + (uint16_t)pose_count * 4U);
            uint16_t values_off = (uint16_t)(ids_off + servo_count);
            uint16_t values_len = (uint16_t)pose_count * (uint16_t)servo_count * 4U;
            uint16_t total_needed = (uint16_t)(values_off + values_len);
            if (total_needed > len) {
                return false;
            }

            int8_t slot = find_free_cycle_slot();
            if (slot < 0) {
                return false;
            }
            s_cycle_used[slot] = 1;

            for (uint8_t i = 0; i < servo_count; ++i) {
                s_cycle_servo_ids[slot][i] = payload[ids_off + i];
            }
            for (uint8_t p = 0; p < pose_count; ++p) {
                uint32_t dur = 0;
                if (!proto_read_u32_le(payload, len, (uint16_t)(durations_off + p * 4U), &dur)) {
                    release_cycle_slot((uint8_t)slot);
                    return false;
                }
                s_cycle_durations[slot][p] = dur;
            }

            if (mode == 0) {
                for (uint8_t p = 0; p < pose_count; ++p) {
                    s_cycle_pose_pwm_ptrs[slot][p] = s_cycle_pose_pwm[slot][p];
                    for (uint8_t i = 0; i < servo_count; ++i) {
                        uint32_t pwm = 0;
                        uint16_t off = (uint16_t)(values_off + (p * servo_count + i) * 4U);
                        if (!proto_read_u32_le(payload, len, off, &pwm)) {
                            release_cycle_slot((uint8_t)slot);
                            return false;
                        }
                        s_cycle_pose_pwm[slot][p][i] = pwm;
                    }
                }
            } else if (mode == 1) {
                for (uint8_t p = 0; p < pose_count; ++p) {
                    s_cycle_pose_angle_ptrs[slot][p] = s_cycle_pose_angle[slot][p];
                    for (uint8_t i = 0; i < servo_count; ++i) {
                        float angle = 0.0f;
                        uint16_t off = (uint16_t)(values_off + (p * servo_count + i) * 4U);
                        if (!proto_read_f32_le(payload, len, off, &angle)) {
                            release_cycle_slot((uint8_t)slot);
                            return false;
                        }
                        s_cycle_pose_angle[slot][p][i] = angle;
                    }
                }
            } else {
                release_cycle_slot((uint8_t)slot);
                return false;
            }

            int32_t cycle_index = motion_cycle_create(s_cycle_servo_ids[slot],
                                                      servo_count,
                                                      (mode == 0) ? s_cycle_pose_pwm_ptrs[slot] : NULL,
                                                      (mode == 1) ? s_cycle_pose_angle_ptrs[slot] : NULL,
                                                      s_cycle_durations[slot],
                                                      pose_count,
                                                      max_loops);
            if (cycle_index < 0 || cycle_index >= PROTO_CYCLE_MAX_SLOT) {
                release_cycle_slot((uint8_t)slot);
                return false;
            }
            s_cycle_slot_of_index[cycle_index] = (uint8_t)slot;

            uint8_t resp[1];
            resp[0] = (uint8_t)cycle_index;
            protocol_send_state(STATE_CMD_MOTION_CYCLE, resp, (uint16_t)sizeof(resp));
            return true;
        }
        case CYCLE_CMD_START: {
            uint32_t idx = 0;
            if (!proto_read_u32_le(payload, len, 0, &idx)) {
                return false;
            }
            return motion_cycle_start(idx) == 0;
        }
        case CYCLE_CMD_RESTART: {
            uint32_t idx = 0;
            if (!proto_read_u32_le(payload, len, 0, &idx)) {
                return false;
            }
            return motion_cycle_restart(idx) == 0;
        }
        case CYCLE_CMD_PAUSE: {
            uint32_t idx = 0;
            if (!proto_read_u32_le(payload, len, 0, &idx)) {
                return false;
            }
            return motion_cycle_pause(idx) == 0;
        }
        case CYCLE_CMD_RELEASE: {
            uint32_t idx = 0;
            if (!proto_read_u32_le(payload, len, 0, &idx)) {
                return false;
            }
            if (idx < PROTO_CYCLE_MAX_SLOT) {
                release_cycle_slot(s_cycle_slot_of_index[idx]);
            }
            return motion_cycle_release(idx) == 0;
        }
        case CYCLE_CMD_GET_STATUS:
        case CYCLE_CMD_STATUS:
            return false;
        default:
            return false;
    }
}
