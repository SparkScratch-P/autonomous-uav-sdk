/*
 * sdk_api_yaw.h - Heading control.
 *
 * yaw(angle_deg)   - turn by a relative angle from the current heading
 * yaw_to(heading)  - turn to an absolute compass heading (0-359)
 * yaw_rate(degps)  - continuous rotation at a rate, no arrival wait
 */
#ifndef SDK_API_YAW_H
#define SDK_API_YAW_H

#include <math.h>
#include "sdk_state.h"
#include "sdk_log.h"
#include "sdk_fsm.h"
#include "sdk_recorder.h"
#include "sdk_commands.h"
#include "sdk_target.h"
#include "sdk_api_flight.h"

static inline float sdk_deg_to_rad(float deg) { return deg * ((float)M_PI / 180.0f); }
static inline float sdk_normalize_deg(float deg)
{
    while (deg < 0.0f)   deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;
    return deg;
}

static inline bool sdk_check_heading(const void *arg)
{
    float target_deg = *(const float *)arg;
    pthread_mutex_lock(&g_telem_lock);
    float yaw_deg = sdk_normalize_deg(g_telem.yaw_rad * (180.0f / (float)M_PI));
    pthread_mutex_unlock(&g_telem_lock);

    float diff = fabsf(yaw_deg - target_deg);
    if (diff > 180.0f) diff = 360.0f - diff;
    return diff <= SDK_HEADING_TOLERANCE_DEG;
}

/* yaw_to(heading_deg)
 *   Turns to an absolute compass heading and confirms via ATTITUDE. */
static inline bool yaw_to(float heading_deg)
{
    SDK_REQUIRE_STATE(SDK_MASK_FLYING);
    float target = sdk_normalize_deg(heading_deg);
    SDK_LOG("yaw_to(%.1f deg)", target);
    SDK_RECORD("YAW_START", "target_deg=%.1f", target);

    if (get_mode() != MODE_OFFBOARD && !sdk_enter_offboard_hold()) {
        SDK_RECORD("YAW_FAILED", "error=%s", get_last_error_string());
        return false;
    }
    sdk_set_state(SDK_MISSION);

    pthread_mutex_lock(&g_telem_lock);
    float x = g_telem.local_pos.x, y = g_telem.local_pos.y, z = g_telem.local_pos.z;
    pthread_mutex_unlock(&g_telem_lock);

    sdk_target_set_local_position(x, y, z, sdk_deg_to_rad(target));
    bool reached = sdk_wait_until(sdk_check_heading, &target, SDK_MODE_TIMEOUT_MS, 100);
    if (reached) {
        sdk_set_state(SDK_HOLD);
        SDK_RECORD("YAW_DONE", "heading_deg=%.1f", target);
    } else {
        SDK_LOG("yaw_to(): heading not confirmed within timeout");
        SDK_RECORD("YAW_FAILED", "error=%s", get_last_error_string());
    }
    return reached;
}

/* yaw(angle_deg)
 *   Turns by a relative angle from the current heading. */
static inline bool yaw(float angle_deg)
{
    pthread_mutex_lock(&g_telem_lock);
    float current_deg = g_telem.yaw_rad * (180.0f / (float)M_PI);
    pthread_mutex_unlock(&g_telem_lock);
    SDK_LOG("yaw(%.1f deg relative)", angle_deg);
    return yaw_to(current_deg + angle_deg);
}

/* wait_until_heading(heading_deg, timeout_ms) */
static inline bool wait_until_heading(float heading_deg, int timeout_ms)
{
    float target = sdk_normalize_deg(heading_deg);
    return sdk_wait_until(sdk_check_heading, &target, timeout_ms, 100);
}

/* yaw_rate(degps)
 *   Continuous rotation at a fixed rate. Does not block or confirm
 *   arrival - call stop_velocity() or set a new target to stop it. */
static inline bool yaw_rate(float degps)
{
    SDK_REQUIRE_STATE(SDK_MASK_FLYING);
    SDK_LOG("yaw_rate(%.1f deg/s)", degps);
    if (get_mode() != MODE_OFFBOARD && !sdk_enter_offboard_hold()) return false;
    sdk_target_set_yaw_rate(sdk_deg_to_rad(degps));
    return true;
}

#endif /* SDK_API_YAW_H */
