#include "../protocol.h"
#include "../transport/TinyFrame/TinyFrame.h"
#include "tf_uart_port.h"

TF_Result protocol_sys_listener(TinyFrame* tf, TF_Msg* msg)
{
    (void)tf;
    if (msg == NULL) {
        return TF_NEXT;
    }

    proto_cmd_view_t cmd_view;
    if (!proto_parse_cmd(msg->data, msg->len, &cmd_view)) {
        return TF_NEXT;
    }

    if (protocol_sys_handle(cmd_view.cmd, cmd_view.payload, cmd_view.payload_len)) {
        return TF_STAY;
    }

    return TF_NEXT;
}

bool protocol_sys_handle(uint8_t cmd, const uint8_t* payload, uint16_t len)
{
    // Payload format: [cmd][payload...]
    switch (cmd) {
        case SYS_CMD_PING: {
            uint8_t  frame[1U + PROTO_MAX_PAYLOAD];
            uint16_t frame_len = 0U;
            if (!proto_encode_cmd_frame(
                    (uint8_t)SYS_CMD_PONG, payload, len, frame, (uint16_t)sizeof(frame), &frame_len)) {
                return false;
            }
            return tf_uart_port_send_frame(PROTO_TYPE_SYS, frame, frame_len);
        }
        case SYS_CMD_PONG:
            return true;
        case SYS_CMD_HEARTBEAT:
            return true;
        case SYS_CMD_GET_INFO: {
            const char* name = PROTO_DEVICE_NAME;
            uint8_t name_len = 0;
            while (name[name_len] != '\0' && name_len < (uint8_t)(PROTO_MAX_PAYLOAD - 3)) {
                name_len++;
            }
            uint8_t payload_buf[3 + PROTO_MAX_PAYLOAD];
            uint8_t frame[1U + PROTO_MAX_PAYLOAD];
            payload_buf[0] = (uint8_t)PROTO_VERSION_MAJOR;
            payload_buf[1] = (uint8_t)PROTO_VERSION_MINOR;
            payload_buf[2] = name_len;
            for (uint8_t i = 0; i < name_len; ++i) {
                payload_buf[3 + i] = (uint8_t)name[i];
            }

            uint16_t frame_len = 0U;
            if (!proto_encode_cmd_frame((uint8_t)SYS_CMD_INFO,
                                        payload_buf,
                                        (uint16_t)(3U + name_len),
                                        frame,
                                        (uint16_t)sizeof(frame),
                                        &frame_len)) {
                return false;
            }
            return tf_uart_port_send_frame(PROTO_TYPE_SYS, frame, frame_len);
        }
        case SYS_CMD_INFO:
            return true;
        case SYS_CMD_RESET:
            // No platform reset hooked yet.
            return true;
        default:
            return false;
    }
}
