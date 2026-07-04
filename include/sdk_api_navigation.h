/*
 * sdk_api_navigation.h - GPS waypoint navigation.
 *
 * Pass SDK_ALT_KEEP_CURRENT / SDK_SPEED_KEEP_CURRENT (or just call
 * gps_go(lat, lon) via the convenience wrappers below) to leave that
 * axis alone - C has no overloading, so this is the plain-C
 * equivalent of the multiple gps_go() signatures.
 */
#ifndef SDK_API_NAVIGATION_H
#define SDK_API_NAVIGATION_H

#include <math.h>
#include "sdk_state.h"
#include "sdk_log.h"
#include "sdk_fsm.h"
#include "sdk_recorder.h"
#include "sdk_commands.h"
#include "sdk_target.h"
#include "sdk_api_flight.h"
#include "sdk_api_speed.h"
#include "sdk_api_safety.h"

/* sdk_horizontal_distance_m()
 *   Great-circle distance between two lat/lon points, in meters. */
static inline double sdk_horizontal_distance_m(double lat1, double lon1, double lat2, double lon2)
{
    double phi1 = lat1 * M_PI / 180.0;
    double phi2 = lat2 * M_PI / 180.0;
    double dphi = (lat2 - lat1) * M_PI / 180.0;
    double dlambda = (lon2 - lon1) * M_PI / 180.0;

    double a = sin(dphi / 2) * sin(dphi / 2) +
               cos(phi1) * cos(phi2) * sin(dlambda / 2) * sin(dlambda / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return SDK_EARTH_RADIUS_M * c;
}

typedef struct { double lat, lon; float radius_m; } sdk_arrival_arg_t;

static inline bool sdk_check_arrival(const void *arg)
{
    const sdk_arrival_arg_t *a = (const sdk_arrival_arg_t *)arg;
    pthread_mutex_lock(&g_telem_lock);
    bool have_fix = g_telem.have_global_pos;
    double lat = g_telem.lat, lon = g_telem.lon;
    pthread_mutex_unlock(&g_telem_lock);
    if (!have_fix) return false;
    return sdk_horizontal_distance_m(lat, lon, a->lat, a->lon) <= a->radius_m;
}

/* wait_until_arrival(lat, lon, radius_m, timeout_ms) */
static inline bool wait_until_arrival(double lat, double lon, float radius_m, int timeout_ms)
{
    sdk_arrival_arg_t a = { lat, lon, radius_m };
    return sdk_wait_until(sdk_check_arrival, &a, timeout_ms, 250);
}

/* gps_go(lat, lon, alt_m, speed_mps)
 *   Flies to a GPS waypoint under Offboard global-position control.
 *   alt_m: pass SDK_ALT_KEEP_CURRENT to hold current relative altitude.
 *   speed_mps: pass SDK_SPEED_KEEP_CURRENT to leave cruise speed as-is,
 *   otherwise it is applied via MAV_CMD_DO_CHANGE_SPEED before moving.
 *   Confirms arrival within SDK_GPS_GO_DEFAULT_RADIUS_M. */
static inline bool gps_go(double lat, double lon, float alt_m, float speed_mps)
{
    SDK_REQUIRE_STATE(SDK_MASK_FLYING);
    SDK_LOG("gps_go(%.7f, %.7f, alt=%.2f, speed=%.2f)", lat, lon, alt_m, speed_mps);
    SDK_RECORD("NAV_START", "gps lat=%.7f lon=%.7f alt=%.2f speed=%.2f", lat, lon, alt_m, speed_mps);

    if (get_mode() != MODE_OFFBOARD && !sdk_enter_offboard_hold()) {
        SDK_RECORD("NAV_FAILED", "error=%s", get_last_error_string());
        return false;
    }
    sdk_set_state(SDK_MISSION);

    sdk_clamp_to_geofence(&lat, &lon, sdk_horizontal_distance_m);

    if (speed_mps != SDK_SPEED_KEEP_CURRENT) {
        float clamped_speed = sdk_clamp_speed(speed_mps);
        sdk_send_command_and_wait(MAV_CMD_DO_CHANGE_SPEED, 1, clamped_speed, -1, 0, 0, 0, 0,
                                   SDK_COMMAND_ACK_TIMEOUT_MS);
        set_speed(clamped_speed);
    }

    float target_alt_m;
    if (alt_m == SDK_ALT_KEEP_CURRENT) {
        pthread_mutex_lock(&g_telem_lock);
        target_alt_m = g_telem.alt_rel_m;
        pthread_mutex_unlock(&g_telem_lock);
    } else {
        target_alt_m = sdk_clamp_altitude(alt_m);
    }

    sdk_target_set_global_position(lat, lon, target_alt_m, NAN);
    bool reached = wait_until_arrival(lat, lon, SDK_GPS_GO_DEFAULT_RADIUS_M, SDK_LAND_TIMEOUT_MS);
    if (reached) {
        sdk_set_state(SDK_HOLD);
        SDK_RECORD("NAV_DONE", "arrived lat=%.7f lon=%.7f", lat, lon);
    } else {
        SDK_LOG("gps_go(): arrival not confirmed within timeout");
        SDK_RECORD("NAV_FAILED", "error=%s", get_last_error_string());
    }
    return reached;
}

#endif /* SDK_API_NAVIGATION_H */
