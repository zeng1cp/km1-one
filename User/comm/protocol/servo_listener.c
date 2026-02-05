#include "protocol.h"
#include "tinyframe/TinyFrame.h"
#include "motion_engine.h"

TF_Result protocol_servo_listener(TinyFrame* tf, TF_Msg* msg)
{
    (void)tf;
    if (msg == NULL) {
        return TF_NEXT;
    }

    proto_cmd_view_t cmd_view;
    if (!proto_parse_cmd(msg->data, msg->len, &cmd_view)) {
        return TF_NEXT;
    }

    if (protocol_servo_handle(cmd_view.cmd, cmd_view.payload, cmd_view.payload_len)) {
        return TF_STAY;
    }

    return TF_NEXT;
}

bool protocol_servo_handle(uint8_t cmd, const uint8_t* payload, uint16_t len)
{
    // Payload format: [cmd][payload...]
    switch (cmd) {
        case SERVO_CMD_ENABLE: {
            // No explicit enable in current motor layer; sync outputs.
            servo_sync_to_hardware();
            return true;
        }
        case SERVO_CMD_DISABLE: {
            if (len == 1) {
                uint8_t id = payload[0];
                if (id >= MAX_SERVOS) {
                    return false;
                }
                servo_stop(id);
                return true;
            }
            servo_emergency_stop();
            return true;
        }
        case SERVO_CMD_SET_PWM: {
            if (len != 9) {
                return false;
            }
            uint8_t  id = payload[0];
            uint32_t pwm = 0;
            uint32_t duration = 0;
            if (id >= MAX_SERVOS) {
                return false;
            }
            if (!proto_read_u32_le(payload, len, 1, &pwm)) {
                return false;
            }
            if (!proto_read_u32_le(payload, len, 5, &duration)) {
                return false;
            }
            servo_move_pwm(id, pwm, duration);
            return true;
        }
        case SERVO_CMD_SET_POS: {
            if (len != 9) {
                return false;
            }
            uint8_t id = payload[0];
            float   angle = 0.0f;
            uint32_t duration = 0;
            if (id >= MAX_SERVOS) {
                return false;
            }
            if (!proto_read_f32_le(payload, len, 1, &angle)) {
                return false;
            }
            if (!proto_read_u32_le(payload, len, 5, &duration)) {
                return false;
            }
            servo_move_angle(id, angle, duration);
            return true;
        }
        case SERVO_CMD_GET_STATUS: {
            if (len != 1) {
                return false;
            }
            uint8_t id = payload[0];
            if (id >= MAX_SERVOS) {
                return false;
            }
            uint8_t resp[14];
            resp[0] = id;
            resp[1] = (uint8_t)(servo_is_moving(id) ? 1 : 0);
            proto_write_u32_le(resp, 2, servo_get_current_pwm(id));
            proto_write_f32_le(resp, 6, servo_get_target_angle(id));
            proto_write_u32_le(resp, 10, servo_get_remaining_time(id));
            return protocol_send_state(STATE_CMD_SERVO, resp, (uint16_t)sizeof(resp));
        }
        case SERVO_CMD_STATUS:
            return true;
        default:
            return false;
    }
}
