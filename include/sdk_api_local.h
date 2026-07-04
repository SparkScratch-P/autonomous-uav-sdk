/*
 * sdk_api_local.h - Local NED and vehicle-body-frame navigation.
 *
 * "Local" here means the LOCAL_POSITION_NED frame PX4 publishes
 * (origin = wherever the EKF was initialized, usually the arming
 * point). Everything is closed-loop against that same message.
 */
#ifndef SDK_API_LOCAL_H
#define SDK_API_LOCAL_H

#include <math.h>
#include "sdk_state.h"
#include "sdk_log.h"
#include "sdk_fsm.h"
#include "sdk_recorder.h"
#include "sdk_commands.h"
#include "sdk_target.h"
#include "sdk_api_flight.h"

typedef struct { float x, y, z; } sdk_local_point_t;

static inline bool sdk_check_local_arrival(const void *arg)
{
    const sdk_local_point_t *target = (const sdk_local_point_t *)arg;
    pthread_mutex_lock(&g_telem_lock);
    bool valid = g_telem.local_pos.valid;
    float dx = g_telem.local_pos.x - target->x;
    float dy = g_telem.local_pos.y - target->y;
    float dz = g_telem.local_pos.z - target->z;
    pthread_mutex_unlock(&g_telem_lock);
    if (!valid) return false;
    return sqrtf(dx * dx + dy * dy + dz * dz) <= SDK_POSITION_TOLERANCE_M;
}

/* goto_local(x, y, z)
 *   Flies to an absolute point in the LOCAL_POSITION_NED frame and
 *   confirms arrival within SDK_POSITION_TOLERANCE_M. */
static inline bool goto_local(float x, float y, float z)
{
    SDK_REQUIRE_STATE(SDK_MASK_FLYING);
    SDK_LOG("goto_local(%.2f, %.2f, %.2f)", x, y, z);
    SDK_RECORD("NAV_START", "local x=%.2f y=%.2f z=%.2f", x, y, z);
    if (get_mode() != MODE_OFFBOARD && !sdk_enter_offboard_hold()) {
        SDK_RECORD("NAV_FAILED", "error=%s", get_last_error_string());
        return false;
    }
    sdk_set_state(SDK_MISSION);

    sdk_target_set_local_position(x, y, z, NAN);
    sdk_local_point_t target = { x, y, z };
    bool reached = sdk_wait_until(sdk_check_local_arrival, &target, SDK_LAND_TIMEOUT_MS, 200);
    if (reached) {
        sdk_set_state(SDK_HOLD);
        SDK_RECORD("NAV_DONE", "arrived x=%.2f y=%.2f z=%.2f", x, y, z);
    } else {
        SDK_LOG("goto_local(): arrival not confirmed within timeout");
        SDK_RECORD("NAV_FAILED", "error=%s", get_last_error_string());
    }
    return reached;
}

/* move_local(dx, dy, dz)
 *   Relative move from wherever the vehicle currently is. */
static inline bool move_local(float dx, float dy, float dz)
{
    pthread_mutex_lock(&g_telem_lock);
    float x = g_telem.local_pos.x + dx;
    float y = g_telem.local_pos.y + dy;
    float z = g_telem.local_pos.z + dz;
    pthread_mutex_unlock(&g_telem_lock);
    SDK_LOG("move_local(%.2f, %.2f, %.2f)", dx, dy, dz);
    return goto_local(x, y, z);
}

/* goto_body(forward, right, up)
 *   Moves relative to the vehicle's current heading: forward/right in
 *   meters along the body X/Y axes, up in meters (positive = climb). */
static inline bool goto_body(float forward, float right, float up)
{
    pthread_mutex_lock(&g_telem_lock);
    float yaw = g_telem.yaw_rad;
    pthread_mutex_unlock(&g_telem_lock);

    float dn =  forward * cosf(yaw) - right * sinf(yaw);
    float de =  forward * sinf(yaw) + right * cosf(yaw);
    float dd = -up;

    SDK_LOG("goto_body(fwd=%.2f, right=%.2f, up=%.2f)", forward, right, up);
    return move_local(dn, de, dd);
}

#endif /* SDK_API_LOCAL_H */
