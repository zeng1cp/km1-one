#include <stdio.h>

#include "motion_engine.h"
#include "protocol.h"
#include "servo_codec.h"
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

static bool protocol_send_servo_status(uint8_t subcmd, uint8_t id)
{
    uint8_t resp_buf[18];
    proto_servo_status_resp_t resp = {
        .subcmd = subcmd,
        .servo_id = id,
        .moving = (uint8_t)(servo_is_moving(id) ? 1U : 0U),
        .current_pwm = servo_get_current_pwm(id),
        .target_angle = servo_get_target_angle(id),
        .remaining_time = servo_get_remaining_time(id),
    };

    uint16_t resp_len = proto_encode_servo_status_resp(&resp, resp_buf, (uint16_t)sizeof(resp_buf));
    if (resp_len == 0U) {
        return false;
    }
    return protocol_send_state(STATE_CMD_SERVO, resp_buf, resp_len);
}

static void protocol_servo_complete_cb(uint8_t id)
{
    if (id >= MAX_SERVOS) {
        return;
    }
    if ((s_servo_notify_mask & (1U << id)) == 0) {
        return;
    }

    s_servo_notify_mask &= ~(1U << id);
    (void)protocol_send_servo_status((uint8_t)SERVO_CMD_STATUS, id);
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
            proto_servo_set_pwm_req_t req;
            if (!proto_decode_servo_set_pwm_req(payload, len, &req)) {
                return false;
            }
            if (req.id >= MAX_SERVOS) {
                return false;
            }
            SERVO_LOG("id=%u pwm=%lu duration=%lu",
                      (unsigned)req.id,
                      (unsigned long)req.pwm,
                      (unsigned long)req.duration_ms);
            servo_move_pwm(req.id, req.pwm, req.duration_ms, protocol_servo_complete_cb);
            s_servo_notify_mask |= (1U << req.id);
            return true;
        }
        case SERVO_CMD_SET_POS: {
            SERVO_LOG("CMD SET_POS");
            SERVO_DUMP("payload", payload, len);
            proto_servo_set_pos_req_t req;
            if (!proto_decode_servo_set_pos_req(payload, len, &req)) {
                return false;
            }
            if (req.id >= MAX_SERVOS) {
                return false;
            }
            SERVO_LOG("id=%u angle=%.3f duration=%lu",
                      (unsigned)req.id,
                      (double)req.angle,
                      (unsigned long)req.duration_ms);
            servo_move_angle(req.id, req.angle, req.duration_ms, protocol_servo_complete_cb);
            s_servo_notify_mask |= (1U << req.id);
            return true;
        }
        case SERVO_CMD_HOME: {
            SERVO_LOG("CMD HOME");
            SERVO_DUMP("payload", payload, len);
            proto_servo_home_req_t req;
            if (!proto_decode_servo_home_req(payload, len, &req)) {
                return false;
            }

            for (uint8_t i = 0; i < MAX_SERVOS; ++i) {
                servo_move_home(i, req.duration_ms, protocol_servo_complete_cb);
                s_servo_notify_mask |= (1U << i);
            }
            return true;
        }
        case SERVO_CMD_GET_STATUS: {
            SERVO_LOG("CMD GET_STATUS");
            SERVO_DUMP("payload", payload, len);
            uint8_t id = 0;
            if (!proto_decode_servo_id_req(payload, len, &id)) {
                return false;
            }
            if (id >= MAX_SERVOS) {
                return false;
            }
            SERVO_LOG("id=%u", (unsigned)id);
            return protocol_send_servo_status((uint8_t)SERVO_CMD_GET_STATUS, id);
        }
        case SERVO_CMD_STATUS:
            SERVO_LOG("CMD STATUS");
            SERVO_DUMP("payload", payload, len);
            return true;
        default:
            return false;
    }
}
