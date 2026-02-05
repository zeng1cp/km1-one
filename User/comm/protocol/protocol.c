#include "protocol.h"

#include "tf_uart_port.h"
#include "tinyframe/TinyFrame.h"

// Per-type listeners defined in their respective files
extern TF_Result protocol_sys_listener(TinyFrame* tf, TF_Msg* msg);
extern TF_Result protocol_servo_listener(TinyFrame* tf, TF_Msg* msg);
extern TF_Result protocol_motion_listener(TinyFrame* tf, TF_Msg* msg);
extern TF_Result protocol_arm_listener(TinyFrame* tf, TF_Msg* msg);
extern TF_Result protocol_config_listener(TinyFrame* tf, TF_Msg* msg);
extern TF_Result protocol_motion_cycle_listener(TinyFrame* tf, TF_Msg* msg);

bool protocol_init(void)
{
    TinyFrame* tf = (TinyFrame*)tf_uart_port_get_instance();
    if (tf == NULL) {
        return false;
    }

    bool ok = true;
    ok &= TF_AddTypeListener(tf, PROTO_TYPE_SYS, protocol_sys_listener);
    ok &= TF_AddTypeListener(tf, PROTO_TYPE_SERVO, protocol_servo_listener);
    ok &= TF_AddTypeListener(tf, PROTO_TYPE_MOTION, protocol_motion_listener);
    ok &= TF_AddTypeListener(tf, PROTO_TYPE_ARM, protocol_arm_listener);
    ok &= TF_AddTypeListener(tf, PROTO_TYPE_MOTION_CYCLE, protocol_motion_cycle_listener);
    ok &= TF_AddTypeListener(tf, PROTO_TYPE_CONFIG, protocol_config_listener);
    return ok;
}
