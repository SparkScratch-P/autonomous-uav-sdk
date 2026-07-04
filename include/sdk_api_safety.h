/*
 * sdk_api_safety.h - Soft limits enforced entirely on the companion
 * computer side: set_max_speed()/set_max_altitude() clamp values used
 * by set_alt(), takeoff(), gps_go(), and velocity() before they are
 * ever sent to PX4. set_geofence() is a horizontal-radius companion
 * check applied the same way.
 *
 * These are a companion-computer safety net, not a replacement for
 * PX4's own onboard geofence/failsafe parameters - configure those
 * too for real missions.
 */
#ifndef SDK_API_SAFETY_H
#define SDK_API_SAFETY_H

#include <math.h>
#include "sdk_state.h"
#include "sdk_log.h"

typedef struct {
    bool   enabled;
    double center_lat, center_lon;
    float  radius_m;
} sdk_geofence_t;

static pthread_mutex_t g_geofence_lock = PTHREAD_MUTEX_INITIALIZER;
static sdk_geofence_t g_geofence = { .enabled = false };

static inline void set_max_speed(float mps)
{
    SDK_LOG("set_max_speed(%.2f m/s)", mps);
    pthread_mutex_lock(&g_tunables_lock);
    g_max_speed_mps = mps;
    pthread_mutex_unlock(&g_tunables_lock);
}

static inline void set_max_altitude(float meters)
{
    SDK_LOG("set_max_altitude(%.2f m)", meters);
    pthread_mutex_lock(&g_tunables_lock);
    g_max_alt_m = meters;
    pthread_mutex_unlock(&g_tunables_lock);
}

/* set_geofence(center_lat, center_lon, radius_m)
 *   Companion-side horizontal geofence. gps_go() clamps requested
 *   waypoints to this radius around the center point. */
static inline void set_geofence(double center_lat, double center_lon, float radius_m)
{
    SDK_LOG("set_geofence(%.7f, %.7f, r=%.1f m)", center_lat, center_lon, radius_m);
    pthread_mutex_lock(&g_geofence_lock);
    g_geofence.enabled    = true;
    g_geofence.center_lat = center_lat;
    g_geofence.center_lon = center_lon;
    g_geofence.radius_m   = radius_m;
    pthread_mutex_unlock(&g_geofence_lock);
}

static inline float sdk_clamp_speed(float requested_mps)
{
    pthread_mutex_lock(&g_tunables_lock);
    float max = g_max_speed_mps;
    pthread_mutex_unlock(&g_tunables_lock);
    float mag = fabsf(requested_mps);
    if (mag <= max || max <= 0.0f) return requested_mps;
    return (requested_mps < 0.0f ? -max : max);
}

static inline float sdk_clamp_altitude(float requested_m)
{
    pthread_mutex_lock(&g_tunables_lock);
    float max = g_max_alt_m;
    pthread_mutex_unlock(&g_tunables_lock);
    if (max <= 0.0f || requested_m <= max) return requested_m;
    SDK_LOG("safety: altitude %.2f m clamped to max %.2f m", requested_m, max);
    return max;
}

/* sdk_clamp_to_geofence()
 *   If a geofence is active and (lat,lon) falls outside it, returns
 *   the point on the fence boundary along the same bearing instead.
 *   Requires the caller-supplied distance function (sdk_horizontal_distance_m)
 *   since that lives in sdk_api_navigation.h, so this takes it as a
 *   pointer to keep the two files decoupled. */
static inline void sdk_clamp_to_geofence(double *lat, double *lon,
                                          double (*distance_fn)(double, double, double, double))
{
    pthread_mutex_lock(&g_geofence_lock);
    bool enabled = g_geofence.enabled;
    double clat = g_geofence.center_lat, clon = g_geofence.center_lon;
    float radius = g_geofence.radius_m;
    pthread_mutex_unlock(&g_geofence_lock);

    if (!enabled) return;

    double dist = distance_fn(clat, clon, *lat, *lon);
    if (dist <= radius) return;

    /* Scale the offset from center back to the fence radius - good
     * enough at typical mission ranges (a few km), where the local
     * flat-earth approximation error is negligible. */
    double scale = radius / dist;
    double dlat = (*lat - clat) * scale;
    double dlon = (*lon - clon) * scale;
    SDK_LOG("safety: geofence clamped waypoint from %.1f m to %.1f m radius", dist, (double)radius);
    *lat = clat + dlat;
    *lon = clon + dlon;
}

#endif /* SDK_API_SAFETY_H */
