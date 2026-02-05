#include "protocol.h"
#include "tinyframe/TinyFrame.h"
#include "motion_engine.h"
#include "robot_arm_control.h"

TF_Result protocol_arm_listener(TinyFrame* tf, TF_Msg* msg)
{
    (void)tf;
    if (msg == NULL) {
        return TF_NEXT;
    }

    proto_cmd_view_t cmd_view;
    if (!proto_parse_cmd(msg->data, msg->len, &cmd_view)) {
        return TF_NEXT;
    }

    if (protocol_arm_handle(cmd_view.cmd, cmd_view.payload, cmd_view.payload_len)) {
        return TF_STAY;
    }

    return TF_NEXT;
}

bool protocol_arm_handle(uint8_t cmd, const uint8_t* payload, uint16_t len)
{
    // Payload format: [cmd][payload...]
    switch (cmd) {
        case ARM_CMD_HOME: {
            uint32_t duration = 1000;
            if (len == 4) {
                if (!proto_read_u32_le(payload, len, 0, &duration)) {
                    return false;
                }
            } else if (len != 0) {
                return false;
            }
            for (uint8_t id = 0; id < ARM_JOINT_COUNT; ++id) {
                servo_move_home(id, duration);
            }
            return true;
        }
        case ARM_CMD_STOP:
            servo_stop_all();
            return true;
        case ARM_CMD_SET_POSE: {
            // payload: [duration:u32][angles:f32 * ARM_JOINT_COUNT]
            const uint16_t expected = (uint16_t)(4 + ARM_JOINT_COUNT * 4);
            if (len != expected) {
                return false;
            }
            uint32_t duration = 0;
            if (!proto_read_u32_le(payload, len, 0, &duration)) {
                return false;
            }
            float angles[ARM_JOINT_COUNT];
            for (uint8_t i = 0; i < ARM_JOINT_COUNT; ++i) {
                if (!proto_read_f32_le(payload, len, (uint16_t)(4 + i * 4), &angles[i])) {
                    return false;
                }
            }
            uint8_t ids[ARM_JOINT_COUNT];
            for (uint8_t i = 0; i < ARM_JOINT_COUNT; ++i) {
                ids[i] = i;
            }
            servo_move_angle_multiple(ids, angles, ARM_JOINT_COUNT, duration);
            return true;
        }
        case ARM_CMD_GET_STATUS: {
            uint8_t resp[4];
            proto_write_u32_le(resp, 0, servo_get_moving_mask());
            return protocol_send_state(STATE_CMD_ARM, resp, (uint16_t)sizeof(resp));
        }
        case ARM_CMD_STATUS:
            return true;
        default:
            return false;
    }
}
