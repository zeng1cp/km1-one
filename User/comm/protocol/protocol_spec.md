# Protocol Spec (km1-one)

This document describes the UART TinyFrame protocol used by `km1-one`.

## Frame Layout

Each TinyFrame message uses:
- `type` = `proto_type_t`
- `data` = `[cmd][payload...]`

All multi-byte numbers in payload are **little-endian (LE)**.

## Type List

- `PROTO_TYPE_SYS    (0x01)`
- `PROTO_TYPE_SERVO  (0x10)`
- `PROTO_TYPE_MOTION (0x11)`
- `PROTO_TYPE_ARM    (0x12)`
- `PROTO_TYPE_MOTION_CYCLE (0x13)`
- `PROTO_TYPE_STATE  (0xD0)` device -> host
- `PROTO_TYPE_CONFIG (0xE0)`
- `PROTO_TYPE_DEBUG  (0xF0)`

## SYS (type 0x01)

Commands:
- `SYS_CMD_PING (0x01)`: payload arbitrary bytes
- `SYS_CMD_PONG (0x02)`: payload arbitrary bytes
- `SYS_CMD_RESET (0x03)`: no payload
- `SYS_CMD_GET_INFO (0x04)`: no payload
- `SYS_CMD_INFO (0x05)`: response payload format below
- `SYS_CMD_HEARTBEAT (0x06)`: no payload

`SYS_CMD_INFO` payload:
- `[proto_major:u8][proto_minor:u8][name_len:u8][name_bytes...]`

## SERVO (type 0x10)

Commands:
- `SERVO_CMD_ENABLE (0x01)`: no payload
- `SERVO_CMD_DISABLE (0x02)`: optional `[id:u8]` (if absent, stop all)
- `SERVO_CMD_SET_PWM (0x03)`: `[id:u8][pwm:u32][duration_ms:u32]`
- `SERVO_CMD_SET_POS (0x04)`: `[id:u8][angle_deg:f32][duration_ms:u32]`
- `SERVO_CMD_GET_STATUS (0x05)`: `[id:u8]`
- `SERVO_CMD_STATUS (0x06)`: (reserved)

State response (`STATE_CMD_SERVO` payload):
- `[id:u8][moving:u8][current_pwm:u32][target_angle_deg:f32][remaining_ms:u32]`

## MOTION (type 0x11)

Commands:
- `MOTION_CMD_START (0x01)`:
  - payload: `[mode:u8][count:u8][duration_ms:u32][ids...][values...]`
  - `mode=0`: values are `u32 pwm`
  - `mode=1`: values are `f32 angle_deg`
- `MOTION_CMD_STOP (0x02)`: `[group_id:u32]`
- `MOTION_CMD_PAUSE (0x03)`: `[group_id:u32]`
- `MOTION_CMD_RESUME (0x04)`: `[group_id:u32]`
- `MOTION_CMD_SET_PLAN (0x05)`: (reserved)
- `MOTION_CMD_GET_STATUS (0x06)`: `[group_id:u32]`
- `MOTION_CMD_STATUS (0x07)`: (reserved)

State response (`STATE_CMD_MOTION` payload):
- `START` response: `[group_id:u32]`
- `GET_STATUS` response: `[group_id:u32][mask:u32][complete:u8]`

## ARM (type 0x12)

Commands:
- `ARM_CMD_HOME (0x01)`: optional `[duration_ms:u32]` (default 1000)
- `ARM_CMD_STOP (0x02)`: no payload
- `ARM_CMD_SET_POSE (0x03)`: `[duration_ms:u32][angles_deg:f32 * ARM_JOINT_COUNT]`
- `ARM_CMD_GET_STATUS (0x04)`: no payload
- `ARM_CMD_STATUS (0x05)`: (reserved)

State response (`STATE_CMD_ARM` payload):
- `[moving_mask:u32]`

## MOTION_CYCLE (type 0x13)

Commands:
- `CYCLE_CMD_CREATE (0x01)`:
  - payload: `[mode:u8][servo_count:u8][pose_count:u8][max_loops:u32]`
  - then: `[durations_ms:u32 * pose_count][ids:u8 * servo_count]`
  - then: `[values:pose_count * servo_count * 4]`
  - `mode=0`: values are `u32 pwm`
  - `mode=1`: values are `f32 angle_deg`
- `CYCLE_CMD_START (0x02)`: `[cycle_index:u32]`
- `CYCLE_CMD_RESTART (0x03)`: `[cycle_index:u32]`
- `CYCLE_CMD_PAUSE (0x04)`: `[cycle_index:u32]`
- `CYCLE_CMD_RELEASE (0x05)`: `[cycle_index:u32]`
- `CYCLE_CMD_GET_STATUS (0x06)`: (reserved)
- `CYCLE_CMD_STATUS (0x07)`: (reserved)

State response (`STATE_CMD_MOTION_CYCLE` payload):
- `CREATE` response: `[cycle_index:u8]`

## CONFIG (type 0xE0)

Commands:
- `CONFIG_CMD_GET (0x01)`: no payload
- `CONFIG_CMD_SET (0x02)`: (reserved)
- `CONFIG_CMD_SAVE (0x03)`: (reserved)
- `CONFIG_CMD_LOAD (0x04)`: (reserved)
- `CONFIG_CMD_RESET (0x05)`: (reserved)

State response (`STATE_CMD_CONFIG` payload):
- currently empty
