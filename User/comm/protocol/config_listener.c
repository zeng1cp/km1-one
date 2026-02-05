#include "protocol.h"
#include "tinyframe/TinyFrame.h"

TF_Result protocol_config_listener(TinyFrame* tf, TF_Msg* msg)
{
    (void)tf;
    if (msg == NULL) {
        return TF_NEXT;
    }

    proto_cmd_view_t cmd_view;
    if (!proto_parse_cmd(msg->data, msg->len, &cmd_view)) {
        return TF_NEXT;
    }

    if (protocol_config_handle(cmd_view.cmd, cmd_view.payload, cmd_view.payload_len)) {
        return TF_STAY;
    }

    return TF_NEXT;
}

bool protocol_config_handle(uint8_t cmd, const uint8_t* payload, uint16_t len)
{
    (void)payload;
    (void)len;
    switch (cmd) {
        case CONFIG_CMD_GET:
            return protocol_send_state(STATE_CMD_CONFIG, NULL, 0);
        case CONFIG_CMD_SET:
            return true;
        case CONFIG_CMD_SAVE:
            return true;
        case CONFIG_CMD_LOAD:
            return true;
        case CONFIG_CMD_RESET:
            return true;
        default:
            return false;
    }
}
