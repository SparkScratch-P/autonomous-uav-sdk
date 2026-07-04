/*
 * sdk_api_config.h - Mission-wide defaults. Setting these does not
 * move the vehicle by itself; they change what later calls default to.
 */
#ifndef SDK_API_CONFIG_H
#define SDK_API_CONFIG_H

#include "sdk_state.h"
#include "sdk_log.h"

/* set_takeoff_alt(meters)
 *   Default altitude used by takeoff() when called with height_cm <= 0. */
static inline void set_takeoff_alt(float meters)
{
    SDK_LOG("set_takeoff_alt(%.2f m)", meters);
    pthread_mutex_lock(&g_tunables_lock);
    g_takeoff_alt_m = meters;
    pthread_mutex_unlock(&g_tunables_lock);
}

static inline float sdk_get_takeoff_alt(void)
{
    pthread_mutex_lock(&g_tunables_lock);
    float v = g_takeoff_alt_m;
    pthread_mutex_unlock(&g_tunables_lock);
    return v;
}

/* set_cruise_speed(mps)
 *   Same tunable set_speed() writes - provided under this name too so
 *   mission setup code can group all defaults together. */
static inline void set_cruise_speed(float mps)
{
    SDK_LOG("set_cruise_speed(%.2f m/s)", mps);
    pthread_mutex_lock(&g_tunables_lock);
    g_cruise_speed_mps = mps;
    pthread_mutex_unlock(&g_tunables_lock);
}

/* set_yaw_speed(degps)
 *   Informational default for mission code that wants a standard
 *   turn rate to pass to yaw_rate() - PX4's own attitude controller
 *   decides actual yaw dynamics for yaw()/yaw_to(), this value is not
 *   force-applied to those. */
static inline void set_yaw_speed(float degps)
{
    SDK_LOG("set_yaw_speed(%.2f deg/s)", degps);
    pthread_mutex_lock(&g_tunables_lock);
    g_yaw_speed_degps = degps;
    pthread_mutex_unlock(&g_tunables_lock);
}

#endif /* SDK_API_CONFIG_H */
