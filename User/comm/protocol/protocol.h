#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#include "protocol_codec.h"

#ifdef __cplusplus
extern "C" {
#endif

// Limits
#define PROTO_MAX_PAYLOAD 256

// Protocol metadata
#define PROTO_VERSION_MAJOR 1
#define PROTO_VERSION_MINOR 0
#define PROTO_DEVICE_NAME   "km1-one"

// Protocol frame types (TinyFrame msg->type)
typedef enum {
    PROTO_TYPE_SYS    = 0x01,
    PROTO_TYPE_SERVO  = 0x10,
    PROTO_TYPE_MOTION = 0x11,
    PROTO_TYPE_CYCLE  = 0x12,
    PROTO_TYPE_ARM    = 0x13,
    PROTO_TYPE_STATE  = 0xD0,
    PROTO_TYPE_CONFIG = 0xE0,
    PROTO_TYPE_DEBUG  = 0xF0,
} proto_type_t;

// SYS commands
typedef enum {
    SYS_CMD_PING      = 0x01,
    SYS_CMD_PONG      = 0x02,
    SYS_CMD_RESET     = 0x03,
    SYS_CMD_GET_INFO  = 0x04,
    SYS_CMD_INFO      = 0x05,
    SYS_CMD_HEARTBEAT = 0x06,
} proto_sys_cmd_t;

// SERVO commands
typedef enum {
    SERVO_CMD_ENABLE     = 0x01,
    SERVO_CMD_DISABLE    = 0x02,
    SERVO_CMD_SET_PWM    = 0x03,
    SERVO_CMD_SET_POS    = 0x04,
    SERVO_CMD_GET_STATUS = 0x05,
    SERVO_CMD_STATUS     = 0x06,
    SERVO_CMD_HOME       = 0x07,
} proto_servo_cmd_t;

// MOTION commands
typedef enum {
    MOTION_CMD_START      = 0x01,
    MOTION_CMD_STOP       = 0x02,
    MOTION_CMD_PAUSE      = 0x03,
    MOTION_CMD_RESUME     = 0x04,
    MOTION_CMD_SET_PLAN   = 0x05,
    MOTION_CMD_GET_STATUS = 0x06,
    MOTION_CMD_STATUS     = 0x07,
} proto_motion_cmd_t;

// CYCLE commands
typedef enum {
    CYCLE_CMD_CREATE     = 0x00,
    CYCLE_CMD_START      = 0x01,
    CYCLE_CMD_RESTART    = 0x02,
    CYCLE_CMD_PAUSE      = 0x03,
    CYCLE_CMD_RELEASE    = 0x04,
    CYCLE_CMD_GET_STATUS = 0x05,
    CYCLE_CMD_STATUS     = 0x06,
    CYCLE_CMD_LIST       = 0x07,
} proto_cycle_cmd_t;

// ARM commands
typedef enum {
    ARM_CMD_HOME       = 0x01,
    ARM_CMD_STOP       = 0x02,
    ARM_CMD_SET_POSE   = 0x03,
    ARM_CMD_GET_STATUS = 0x04,
    ARM_CMD_STATUS     = 0x05,
} proto_arm_cmd_t;

// CONFIG commands
typedef enum {
    CONFIG_CMD_GET   = 0x01,
    CONFIG_CMD_SET   = 0x02,
    CONFIG_CMD_SAVE  = 0x03,
    CONFIG_CMD_LOAD  = 0x04,
    CONFIG_CMD_RESET = 0x05,
} proto_config_cmd_t;

// STATE commands (device -> host)
typedef enum {
    STATE_CMD_SYS    = 0x01,
    STATE_CMD_SERVO  = 0x02,
    STATE_CMD_MOTION = 0x03,
    STATE_CMD_CYCLE  = 0x04,
    STATE_CMD_ARM    = 0x05,
    STATE_CMD_CONFIG = 0x06,
} proto_state_cmd_t;

// Listener entry points
bool protocol_sys_handle(uint8_t cmd, const uint8_t* payload, uint16_t len);
bool protocol_servo_handle(uint8_t cmd, const uint8_t* payload, uint16_t len);
bool protocol_motion_handle(uint8_t cmd, const uint8_t* payload, uint16_t len);
bool protocol_cycle_handle(uint8_t cmd, const uint8_t* payload, uint16_t len);
bool protocol_arm_handle(uint8_t cmd, const uint8_t* payload, uint16_t len);
bool protocol_config_handle(uint8_t cmd, const uint8_t* payload, uint16_t len);

// State sender
bool protocol_send_state(uint8_t cmd, const uint8_t* payload, uint16_t len);

// Register type listeners with TinyFrame instance
bool protocol_init(void);

#ifdef __cplusplus
}
#endif

#endif  // PROTOCOL_H
