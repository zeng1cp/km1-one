#include "../protocol.h"
#include "../transport/TinyFrame/TinyFrame.h"
#include <string.h>
#include "arm_codec.h"
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

    (void)protocol_arm_handle(cmd_view.cmd, cmd_view.payload, cmd_view.payload_len);
    return TF_STAY;
}

bool protocol_arm_handle(uint8_t cmd, const uint8_t* payload, uint16_t len)
{
    // Payload format: [cmd][payload...]
    switch (cmd) {
        case ARM_CMD_HOME: {
            proto_arm_home_req_t req;
            if (!proto_decode_arm_home_req(payload, len, 1000U, &req)) {
                return false;
            }
            for (uint8_t id = 0; id < ARM_JOINT_COUNT; ++id) {
                servo_move_home(id, req.duration_ms, NULL);
            }
            return true;
        }
        case ARM_CMD_STOP:
            servo_stop_all();
            return true;
        case ARM_CMD_SET_POSE: {
            // Payload format: [duration:u32][angles:f32 * ARM_JOINT_COUNT]
            proto_arm_set_pose_req_t req;
            if (!proto_decode_arm_set_pose_req(payload, len, ARM_JOINT_COUNT, &req)) {
                return false;
            }
            float angles[ARM_JOINT_COUNT];
            for (uint8_t i = 0; i < ARM_JOINT_COUNT; ++i) {
                memcpy(&angles[i], &req.angles_raw[(uint16_t)i * 4U], sizeof(float));
            }
            uint8_t ids[ARM_JOINT_COUNT];
            for (uint8_t i = 0; i < ARM_JOINT_COUNT; ++i) {
                ids[i] = i;
            }
            servo_move_angle_multiple(ids, angles, ARM_JOINT_COUNT, req.duration_ms, NULL);
            return true;
        }
        case ARM_CMD_GET_STATUS: {
            uint8_t resp_buf[4];
            proto_arm_status_resp_t resp = {
                .moving_mask = servo_get_moving_mask(),
            };
            uint16_t resp_len =
                proto_encode_arm_status_resp(&resp, resp_buf, (uint16_t)sizeof(resp_buf));
            if (resp_len == 0U) {
                return false;
            }
            return protocol_send_state(STATE_CMD_ARM, resp_buf, resp_len);
        }
        case ARM_CMD_STATUS:
            return true;
        default:
            return false;
    }
}
