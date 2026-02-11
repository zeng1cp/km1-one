#include <stdio.h>

#include "motion_engine.h"
#include "protocol.h"
#include "tinyframe/TinyFrame.h"

#ifndef SERVO_LISTENER_LOG_ENABLE
#define SERVO_LISTENER_LOG_ENABLE 1
#endif

#if SERVO_LISTENER_LOG_ENABLE
#define SERVO_LOG(fmt, ...) printf("[servo] " fmt "\n", ##__VA_ARGS__)
#define SERVO_DUMP(label, data, len)                            \
    do {                                                        \
        printf("[servo] %s (len=%u):", label, (unsigned)(len)); \
        for (uint16_t _i = 0; _i < (len); ++_i) {               \
            printf(" %02X", (unsigned)(data)[_i]);              \
        }                                                       \
        printf("\n");                                           \
    } while (0)
#else
#define SERVO_LOG(fmt, ...)          ((void)0)
#define SERVO_DUMP(label, data, len) ((void)0)
#endif

static uint32_t s_servo_notify_mask = 0;

static void protocol_servo_complete_cb(uint8_t id)
{
    if (id >= MAX_SERVOS) {
        return;
    }
    if ((s_servo_notify_mask & (1U << id)) == 0) {
        return;
    }

    s_servo_notify_mask &= ~(1U << id);

    uint8_t resp[18];
    resp[0] = (uint8_t)SERVO_CMD_STATUS;
    proto_write_u32_le(resp, 1, id);
    resp[5] = (uint8_t)(servo_is_moving(id) ? 1 : 0);
    proto_write_u32_le(resp, 6, servo_get_current_pwm(id));
    proto_write_f32_le(resp, 10, servo_get_target_angle(id));
    proto_write_u32_le(resp, 14, servo_get_remaining_time(id));
    protocol_send_state(STATE_CMD_SERVO, resp, (uint16_t)sizeof(resp));
}

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

    (void)protocol_servo_handle(cmd_view.cmd, cmd_view.payload, cmd_view.payload_len);
    return TF_STAY;
}

bool protocol_servo_handle(uint8_t cmd, const uint8_t* payload, uint16_t len)
{
    // Payload format: [cmd][payload...]
    switch (cmd) {
        case SERVO_CMD_ENABLE: {
            SERVO_LOG("CMD ENABLE");
            SERVO_DUMP("payload", payload, len);
            // No explicit enable in current motor layer; sync outputs.
            servo_sync_to_hardware();
            return true;
        }
        case SERVO_CMD_DISABLE: {
            SERVO_LOG("CMD DISABLE");
            SERVO_DUMP("payload", payload, len);
            if (len == 1) {
                uint8_t id = payload[0];
                if (id >= MAX_SERVOS) {
                    return false;
                }
                SERVO_LOG("id=%u", (unsigned)id);
                servo_stop(id);
                return true;
            }
            servo_emergency_stop();
            return true;
        }
        case SERVO_CMD_SET_PWM: {
            SERVO_LOG("CMD SET_PWM");
            SERVO_DUMP("payload", payload, len);
            if (len != 9) {
                return false;
            }
            uint8_t  id       = payload[0];
            uint32_t pwm      = 0;
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
            SERVO_LOG("id=%u pwm=%lu duration=%lu",
                      (unsigned)id,
                      (unsigned long)pwm,
                      (unsigned long)duration);
            servo_move_pwm(id, pwm, duration, protocol_servo_complete_cb);
            s_servo_notify_mask |= (1U << id);
            return true;
        }
        case SERVO_CMD_SET_POS: {
            SERVO_LOG("CMD SET_POS");
            SERVO_DUMP("payload", payload, len);
            if (len != 9) {
                return false;
            }
            uint8_t  id       = payload[0];
            float    angle    = 0.0f;
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
            SERVO_LOG("id=%u angle=%.3f duration=%lu",
                      (unsigned)id,
                      (double)angle,
                      (unsigned long)duration);
            servo_move_angle(id, angle, duration, protocol_servo_complete_cb);
            s_servo_notify_mask |= (1U << id);
            return true;
        }
        case SERVO_CMD_GET_STATUS: {
            SERVO_LOG("CMD GET_STATUS");
            SERVO_DUMP("payload", payload, len);
            if (len != 1) {
                return false;
            }
            uint8_t id = payload[0];
            if (id >= MAX_SERVOS) {
                return false;
            }
            SERVO_LOG("id=%u", (unsigned)id);
            uint8_t resp[18];
            resp[0] = (uint8_t)SERVO_CMD_GET_STATUS;
            proto_write_u32_le(resp, 1, id);
            resp[5] = (uint8_t)(servo_is_moving(id) ? 1 : 0);
            proto_write_u32_le(resp, 6, servo_get_current_pwm(id));
            proto_write_f32_le(resp, 10, servo_get_target_angle(id));
            proto_write_u32_le(resp, 14, servo_get_remaining_time(id));
            return protocol_send_state(STATE_CMD_SERVO, resp, (uint16_t)sizeof(resp));
        }
        case SERVO_CMD_STATUS:
            SERVO_LOG("CMD STATUS");
            SERVO_DUMP("payload", payload, len);
            return true;
        default:
            return false;
    }
}
