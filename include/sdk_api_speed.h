/*
 * sdk_api_speed.h - Cruise-speed tunable and speed telemetry.
 *
 * There is no true airspeed sensor on a multirotor, so
 * get_air_speed() reports the same ground-relative speed as
 * get_speed() - it exists for API symmetry with fixed-wing users of
 * this SDK, not because the two differ on a copter.
 */
#ifndef SDK_API_SPEED_H
#define SDK_API_SPEED_H

#include <math.h>
#include "sdk_state.h"
#include "sdk_log.h"
#include "sdk_commands.h"

/* set_speed(mps)
 *   Sets the cruise speed used by navigation functions (gps_go(),
 *   goto_local(), ...). Does not by itself move the vehicle. */
static inline void set_speed(float mps)
{
    SDK_LOG("set_speed(%.2f m/s)", mps);
    pthread_mutex_lock(&g_tunables_lock);
    g_cruise_speed_mps = mps;
    pthread_mutex_unlock(&g_tunables_lock);
}

/* get_speed() / get_ground_speed()
 *   Horizontal speed magnitude in m/s, from LOCAL_POSITION_NED. */
static inline float get_ground_speed(void)
{
    pthread_mutex_lock(&g_telem_lock);
    float vx = g_telem.local_pos.vx, vy = g_telem.local_pos.vy;
    pthread_mutex_unlock(&g_telem_lock);
    return sqrtf(vx * vx + vy * vy);
}

static inline float get_speed(void)
{
    return get_ground_speed();
}

static inline float get_air_speed(void)
{
    return get_ground_speed(); /* see file header note */
}

/* get_vertical_speed()
 *   m/s, positive = descending (NED convention). */
static inline float get_vertical_speed(void)
{
    pthread_mutex_lock(&g_telem_lock);
    float v = g_telem.local_pos.vz;
    pthread_mutex_unlock(&g_telem_lock);
    return v;
}

static inline bool sdk_check_speed(const void *arg)
{
    float target = *(const float *)arg;
    return fabsf(get_ground_speed() - target) <= SDK_SPEED_TOLERANCE_MPS;
}

static inline bool wait_until_speed(float target_mps, int timeout_ms)
{
    return sdk_wait_until(sdk_check_speed, &target_mps, timeout_ms, 200);
}

#endif /* SDK_API_SPEED_H */
