#include "protocol.h"
#include "tf_uart_port.h"

bool protocol_send_state(uint8_t cmd, const uint8_t* payload, uint16_t len)
{
    uint8_t  frame[1U + PROTO_MAX_PAYLOAD];
    uint16_t frame_len = 0U;

    if (!proto_encode_cmd_frame(cmd, payload, len, frame, (uint16_t)sizeof(frame), &frame_len)) {
        return false;
    }
    return tf_uart_port_send_frame(PROTO_TYPE_STATE, frame, frame_len);
}
