/*
 * sdk_target.h - The single "what should Offboard be streaming right
 * now" slot, and the functions that update it.
 *
 * The streaming thread (sdk_threads.h) just reads this every tick and
 * sends whatever is here. API calls (takeoff, gps_go, velocity, ...)
 * only ever need to update this struct - they never touch the socket.
 */
#ifndef SDK_TARGET_H
#define SDK_TARGET_H

#include <math.h>
#include "sdk_state.h"

static inline void sdk_target_set_local_position(float x, float y, float z, float yaw_rad)
{
    pthread_mutex_lock(&g_target_lock);
    g_target.kind = TARGET_LOCAL_POSITION;
    g_target.x = x;
    g_target.y = y;
    g_target.z = z;
    g_target.yaw_rad = yaw_rad;
    g_target.has_yaw = !isnan(yaw_rad);
    pthread_mutex_unlock(&g_target_lock);
}

static inline void sdk_target_set_local_velocity(float vx, float vy, float vz, float yaw_rad)
{
    pthread_mutex_lock(&g_target_lock);
    g_target.kind = TARGET_LOCAL_VELOCITY;
    g_target.x = vx;
    g_target.y = vy;
    g_target.z = vz;
    g_target.yaw_rad = yaw_rad;
    g_target.has_yaw = !isnan(yaw_rad);
    pthread_mutex_unlock(&g_target_lock);
}

static inline void sdk_target_set_global_position(double lat, double lon, float alt_m, float yaw_rad)
{
    pthread_mutex_lock(&g_target_lock);
    g_target.kind = TARGET_GLOBAL_POSITION;
    g_target.lat = lat;
    g_target.lon = lon;
    g_target.alt_m = alt_m;
    g_target.yaw_rad = yaw_rad;
    g_target.has_yaw = !isnan(yaw_rad);
    pthread_mutex_unlock(&g_target_lock);
}

/* sdk_target_set_yaw_rate()
 *   rate_rad_s is stashed in the .x field of the shared target - the
 *   streaming thread knows to reinterpret it for TARGET_YAW_RATE. */
static inline void sdk_target_set_yaw_rate(float rate_rad_s)
{
    pthread_mutex_lock(&g_target_lock);
    g_target.kind = TARGET_YAW_RATE;
    g_target.x = rate_rad_s;
    g_target.has_yaw = false;
    pthread_mutex_unlock(&g_target_lock);
}

/* target_clear()
 *   Stops the streaming thread from sending anything (it will simply
 *   idle). Used on shutdown and when autonomy is disabled. */
static inline void target_clear(void)
{
    pthread_mutex_lock(&g_target_lock);
    g_target.kind = TARGET_NONE;
    pthread_mutex_unlock(&g_target_lock);
}

#endif /* SDK_TARGET_H */
