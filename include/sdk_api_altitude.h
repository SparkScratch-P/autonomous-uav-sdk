/*
 * sdk_api_altitude.h - Altitude setpoints and readback, closed-loop
 * via GLOBAL_POSITION_INT's relative altitude.
 */
#ifndef SDK_API_ALTITUDE_H
#define SDK_API_ALTITUDE_H

#include "sdk_state.h"
#include "sdk_log.h"
#include "sdk_commands.h"
#include "sdk_target.h"
#include "sdk_api_flight.h"
#include "sdk_api_safety.h"

/* set_alt(height_cm)
 *   Climbs or descends to an absolute relative altitude (above home),
 *   holding current XY position, and confirms arrival via telemetry. */
static inline bool set_alt(int height_cm)
{
    SDK_REQUIRE_STATE(SDK_MASK_FLYING);
    float target_alt_m = sdk_clamp_altitude((float)height_cm / 100.0f);
    SDK_LOG("set_alt(%d cm) -> clamped target %.2f m", height_cm, target_alt_m);

    if (get_mode() != MODE_OFFBOARD && !sdk_enter_offboard_hold()) return false;

    pthread_mutex_lock(&g_telem_lock);
    float x = g_telem.local_pos.x, y = g_telem.local_pos.y;
    pthread_mutex_unlock(&g_telem_lock);

    sdk_target_set_local_position(x, y, -target_alt_m, NAN);
    return sdk_wait_until(sdk_check_alt_rel, &target_alt_m, SDK_LAND_TIMEOUT_MS, 200);
}

/* change_alt(delta_cm)
 *   Relative altitude change from wherever the vehicle currently is. */
static inline bool change_alt(int delta_cm)
{
    pthread_mutex_lock(&g_telem_lock);
    float current_alt_m = g_telem.alt_rel_m;
    pthread_mutex_unlock(&g_telem_lock);

    int target_cm = (int)(current_alt_m * 100.0f) + delta_cm;
    SDK_LOG("change_alt(%d cm) -> target %d cm", delta_cm, target_cm);
    return set_alt(target_cm);
}

static inline bool climb(int cm)   { SDK_LOG("climb(%d cm)", cm);   return change_alt(cm); }
static inline bool descend(int cm) { SDK_LOG("descend(%d cm)", cm); return change_alt(-cm); }

/* get_altitude()
 *   Absolute altitude above mean sea level, in meters. */
static inline float get_altitude(void)
{
    pthread_mutex_lock(&g_telem_lock);
    float v = g_telem.alt_msl_m;
    pthread_mutex_unlock(&g_telem_lock);
    return v;
}

/* get_relative_altitude()
 *   Altitude above the home/takeoff point, in meters. */
static inline float get_relative_altitude(void)
{
    pthread_mutex_lock(&g_telem_lock);
    float v = g_telem.alt_rel_m;
    pthread_mutex_unlock(&g_telem_lock);
    return v;
}

/* wait_until_altitude(height_cm, timeout_ms)
 *   Blocks until relative altitude is within tolerance of height_cm,
 *   without sending a new setpoint (use with set_alt()/takeoff() when
 *   you want to wait separately from commanding). */
static inline bool wait_until_altitude(int height_cm, int timeout_ms)
{
    float target_alt_m = (float)height_cm / 100.0f;
    return sdk_wait_until(sdk_check_alt_rel, &target_alt_m, timeout_ms, 200);
}

#endif /* SDK_API_ALTITUDE_H */
