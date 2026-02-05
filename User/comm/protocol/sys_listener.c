#include "protocol.h"
#include "tinyframe/TinyFrame.h"
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
            uint8_t buf[1 + PROTO_MAX_PAYLOAD];
            if (len > PROTO_MAX_PAYLOAD) {
                return false;
            }
            buf[0] = SYS_CMD_PONG;
            for (uint16_t i = 0; i < len; ++i) {
                buf[1 + i] = payload[i];
            }
            return tf_uart_port_send_frame(PROTO_TYPE_SYS, buf, (uint16_t)(1 + len));
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
            uint8_t buf[1 + 3 + PROTO_MAX_PAYLOAD];
            buf[0] = SYS_CMD_INFO;
            buf[1] = (uint8_t)PROTO_VERSION_MAJOR;
            buf[2] = (uint8_t)PROTO_VERSION_MINOR;
            buf[3] = name_len;
            for (uint8_t i = 0; i < name_len; ++i) {
                buf[4 + i] = (uint8_t)name[i];
            }
            return tf_uart_port_send_frame(PROTO_TYPE_SYS, buf, (uint16_t)(4 + name_len));
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
