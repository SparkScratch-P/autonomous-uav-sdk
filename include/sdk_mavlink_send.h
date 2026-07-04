/*
 * sdk_mavlink_send.h - Builds and sends outbound MAVLink messages.
 *
 * These are thin wrappers: encode one message, hand it to
 * sdk_serial_send(). Anything that needs to *wait* for a result lives
 * in sdk_commands.h instead.
 */
#ifndef SDK_MAVLINK_SEND_H
#define SDK_MAVLINK_SEND_H

#include <math.h>
#include "sdk_state.h"
#include "sdk_serial.h"

static inline void sdk_send_heartbeat(void)
{
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(SDK_SYSTEM_ID, SDK_COMPONENT_ID, &msg,
                                MAV_TYPE_ONBOARD_CONTROLLER,
                                MAV_AUTOPILOT_INVALID,
                                0, 0, MAV_STATE_ACTIVE);
    sdk_serial_send(&msg);
}

/* sdk_send_command_long()
 *   Fire-and-forget send of a COMMAND_LONG. Use sdk_send_command_and_wait()
 *   in sdk_commands.h if you need the ACK. */
static inline void sdk_send_command_long(uint16_t command,
                                          float p1, float p2, float p3, float p4,
                                          float p5, float p6, float p7)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(SDK_SYSTEM_ID, SDK_COMPONENT_ID, &msg,
                                   g_target_sysid, g_target_compid,
                                   command, 0,
                                   p1, p2, p3, p4, p5, p6, p7);
    sdk_serial_send(&msg);
}

static inline void sdk_send_set_mode(uint32_t px4_custom_mode)
{
    mavlink_message_t msg;
    mavlink_msg_set_mode_pack(SDK_SYSTEM_ID, SDK_COMPONENT_ID, &msg,
                               g_target_sysid,
                               MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                               px4_custom_mode);
    sdk_serial_send(&msg);
}

/* --- Offboard setpoints -------------------------------------------- */
/* type_mask bits per SET_POSITION_TARGET_LOCAL_NED: 1 = ignore that field. */
#define SDK_POS_MASK_USE_POSITION_YAW \
    (POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE | \
     POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE | \
     POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)

#define SDK_POS_MASK_USE_VELOCITY_YAW \
    (POSITION_TARGET_TYPEMASK_X_IGNORE | POSITION_TARGET_TYPEMASK_Y_IGNORE | POSITION_TARGET_TYPEMASK_Z_IGNORE | \
     POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE | \
     POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)

/* sdk_send_setpoint_local_position()
 *   x,y,z in meters NED relative to the local origin. yaw_rad may be
 *   NAN to hold current heading (yaw rate ignored, yaw ignored too -
 *   PX4 then just keeps flying its last commanded yaw). */
static inline void sdk_send_setpoint_local_position(float x, float y, float z, float yaw_rad)
{
    uint16_t mask = SDK_POS_MASK_USE_POSITION_YAW;
    if (isnan(yaw_rad)) mask |= POSITION_TARGET_TYPEMASK_YAW_IGNORE;

    mavlink_message_t msg;
    mavlink_msg_set_position_target_local_ned_pack(
        SDK_SYSTEM_ID, SDK_COMPONENT_ID, &msg,
        0, g_target_sysid, g_target_compid,
        MAV_FRAME_LOCAL_NED, mask,
        x, y, z,
        0, 0, 0,
        0, 0, 0,
        isnan(yaw_rad) ? 0.0f : yaw_rad, 0.0f);
    sdk_serial_send(&msg);
}

/* sdk_send_setpoint_local_velocity()
 *   vx,vy,vz in m/s NED. */
static inline void sdk_send_setpoint_local_velocity(float vx, float vy, float vz, float yaw_rad)
{
    uint16_t mask = SDK_POS_MASK_USE_VELOCITY_YAW;
    if (isnan(yaw_rad)) mask |= POSITION_TARGET_TYPEMASK_YAW_IGNORE;

    mavlink_message_t msg;
    mavlink_msg_set_position_target_local_ned_pack(
        SDK_SYSTEM_ID, SDK_COMPONENT_ID, &msg,
        0, g_target_sysid, g_target_compid,
        MAV_FRAME_LOCAL_NED, mask,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        isnan(yaw_rad) ? 0.0f : yaw_rad, 0.0f);
    sdk_serial_send(&msg);
}

/* sdk_send_setpoint_yaw_rate()
 *   Commands only a yaw rotation rate (rad/s), holding position/
 *   velocity/acceleration and absolute yaw all ignored. */
static inline void sdk_send_setpoint_yaw_rate(float yaw_rate_rad_s)
{
    uint16_t mask = POSITION_TARGET_TYPEMASK_X_IGNORE  | POSITION_TARGET_TYPEMASK_Y_IGNORE  | POSITION_TARGET_TYPEMASK_Z_IGNORE  |
                    POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                    POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                    POSITION_TARGET_TYPEMASK_YAW_IGNORE;

    mavlink_message_t msg;
    mavlink_msg_set_position_target_local_ned_pack(
        SDK_SYSTEM_ID, SDK_COMPONENT_ID, &msg,
        0, g_target_sysid, g_target_compid,
        MAV_FRAME_LOCAL_NED, mask,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0.0f, yaw_rate_rad_s);
    sdk_serial_send(&msg);
}

/* sdk_send_setpoint_global_position()
 *   lat/lon in degrees, alt_m relative to home (MAV_FRAME_GLOBAL_RELATIVE_ALT_INT). */
static inline void sdk_send_setpoint_global_position(double lat, double lon, float alt_m, float yaw_rad)
{
    uint16_t mask = SDK_POS_MASK_USE_POSITION_YAW;
    if (isnan(yaw_rad)) mask |= POSITION_TARGET_TYPEMASK_YAW_IGNORE;

    mavlink_message_t msg;
    mavlink_msg_set_position_target_global_int_pack(
        SDK_SYSTEM_ID, SDK_COMPONENT_ID, &msg,
        0, g_target_sysid, g_target_compid,
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, mask,
        (int32_t)(lat * 1e7), (int32_t)(lon * 1e7), alt_m,
        0, 0, 0,
        0, 0, 0,
        isnan(yaw_rad) ? 0.0f : yaw_rad, 0.0f);
    sdk_serial_send(&msg);
}

#endif /* SDK_MAVLINK_SEND_H */
