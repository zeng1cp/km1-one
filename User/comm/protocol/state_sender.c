#include "protocol.h"
#include "tf_uart_port.h"

bool protocol_send_state(uint8_t cmd, const uint8_t* payload, uint16_t len)
{
    if (len > PROTO_MAX_PAYLOAD) {
        return false;
    }

    if (payload == NULL || len == 0) {
        uint8_t buf[1];
        buf[0] = cmd;
        return tf_uart_port_send_frame(PROTO_TYPE_STATE, buf, 1);
    }

    // Payload format: [cmd][payload...]
    uint8_t buf[1 + len];
    buf[0] = cmd;
    for (uint16_t i = 0; i < len; ++i) {
        buf[1 + i] = payload[i];
    }

    return tf_uart_port_send_frame(PROTO_TYPE_STATE, buf, (uint16_t)(1 + len));
}
