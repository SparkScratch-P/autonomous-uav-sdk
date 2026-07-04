/*
 * sdk_api_velocity.h - Direct velocity control. Unlike goto_local()/
 * gps_go(), these do not wait for arrival - they are for continuous
 * control loops (e.g. vision-based tracking on the Jetson) that call
 * velocity() every cycle with a fresh vector.
 */
#ifndef SDK_API_VELOCITY_H
#define SDK_API_VELOCITY_H

#include <math.h>
#include "sdk_state.h"
#include "sdk_log.h"
#include "sdk_fsm.h"
#include "sdk_recorder.h"
#include "sdk_target.h"
#include "sdk_api_flight.h"
#include "sdk_api_safety.h"

/* velocity(vx, vy, vz)
 *   Commands a velocity vector in the LOCAL_POSITION_NED frame
 *   (m/s). Streams continuously until replaced or cleared. */
static inline bool velocity(float vx, float vy, float vz)
{
    SDK_REQUIRE_STATE(SDK_MASK_FLYING);
    if (get_mode() != MODE_OFFBOARD && !sdk_enter_offboard_hold()) return false;

    float horiz_mag = sqrtf(vx * vx + vy * vy);
    float clamped_mag = sdk_clamp_speed(horiz_mag);
    if (clamped_mag != horiz_mag && horiz_mag > 0.0f) {
        float scale = clamped_mag / horiz_mag;
        vx *= scale;
        vy *= scale;
    }

    sdk_target_set_local_velocity(vx, vy, vz, NAN);
    sdk_set_state(SDK_MISSION);
    SDK_RECORD("VELOCITY", "vx=%.2f vy=%.2f vz=%.2f", vx, vy, vz);
    return true;
}

/* velocity_body(forward, right, down)
 *   Same as velocity(), but expressed in the vehicle's current body
 *   frame instead of NED. */
static inline bool velocity_body(float forward, float right, float down)
{
    pthread_mutex_lock(&g_telem_lock);
    float yaw = g_telem.yaw_rad;
    pthread_mutex_unlock(&g_telem_lock);

    float vn =  forward * cosf(yaw) - right * sinf(yaw);
    float ve =  forward * sinf(yaw) + right * cosf(yaw);
    return velocity(vn, ve, down);
}

/* stop_velocity()
 *   Commands zero velocity (hover in place) without leaving Offboard. */
static inline bool stop_velocity(void)
{
    SDK_LOG("stop_velocity()");
    bool ok = velocity(0.0f, 0.0f, 0.0f);
    if (ok) sdk_set_state(SDK_HOLD);
    return ok;
}

#endif /* SDK_API_VELOCITY_H */
