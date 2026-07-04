/*
 * sdk_api_flight.h - Takeoff, landing, RTL, hold, and the emergency
 * brake. Every transition here is confirmed via telemetry, never
 * assumed from elapsed time.
 */
#ifndef SDK_API_FLIGHT_H
#define SDK_API_FLIGHT_H

#include <math.h>
#include "sdk_state.h"
#include "sdk_log.h"
#include "sdk_error.h"
#include "sdk_fsm.h"
#include "sdk_recorder.h"
#include "sdk_time_util.h"
#include "sdk_commands.h"
#include "sdk_target.h"
#include "sdk_api_core.h"
#include "sdk_api_safety.h"
#include "sdk_api_config.h"

/* sdk_enter_offboard_hold()
 *   Starts streaming the vehicle's current position (satisfying PX4's
 *   pre-Offboard streaming requirement), arms if necessary, and
 *   switches to OFFBOARD. Every autonomous-movement function calls
 *   this first so mav.c never has to think about mode sequencing. */
static inline bool sdk_enter_offboard_hold(void)
{
    pthread_mutex_lock(&g_telem_lock);
    bool have_pos = g_telem.local_pos.valid;
    float x = g_telem.local_pos.x, y = g_telem.local_pos.y, z = g_telem.local_pos.z;
    pthread_mutex_unlock(&g_telem_lock);

    if (!have_pos) {
        SDK_LOG("offboard: no LOCAL_POSITION_NED received yet, cannot start");
        sdk_set_error(SDK_NO_TELEMETRY);
        return false;
    }

    sdk_target_set_local_position(x, y, z, NAN);
    sleep_ms(SDK_OFFBOARD_PRESTREAM_MS);

    if (!is_armed() && !arm()) {
        SDK_LOG("offboard: arm() failed");
        return false; /* arm() already set the error code */
    }
    if (get_mode() != MODE_OFFBOARD && !set_mode(MODE_OFFBOARD)) {
        SDK_LOG("offboard: PX4 refused OFFBOARD mode");
        return false; /* set_mode() already set the error code */
    }
    return true;
}

/* takeoff(height_cm)
 *   Climbs to height_cm above the current position under Offboard
 *   position control and confirms arrival via GLOBAL_POSITION_INT's
 *   relative altitude. */
static inline bool takeoff(int height_cm)
{
    SDK_REQUIRE_STATE(SDK_BIT(SDK_READY) | SDK_BIT(SDK_ARMED));

    float requested_m = (height_cm > 0) ? (float)height_cm / 100.0f : sdk_get_takeoff_alt();
    float target_alt_m = sdk_clamp_altitude(requested_m);
    SDK_LOG("takeoff(%d cm) -> clamped target %.2f m", height_cm, target_alt_m);
    SDK_RECORD("TAKEOFF_START", "requested_cm=%d target_m=%.2f", height_cm, target_alt_m);

    if (!sdk_enter_offboard_hold()) {
        SDK_RECORD("TAKEOFF_FAILED", "error=%s", get_last_error_string());
        return false;
    }
    sdk_set_state(SDK_MISSION);

    pthread_mutex_lock(&g_telem_lock);
    float x = g_telem.local_pos.x, y = g_telem.local_pos.y;
    pthread_mutex_unlock(&g_telem_lock);

    sdk_target_set_local_position(x, y, -target_alt_m, NAN);
    bool reached = sdk_wait_until(sdk_check_alt_rel, &target_alt_m, SDK_LAND_TIMEOUT_MS, 200);
    if (reached) {
        sdk_set_state(SDK_HOLD);
        SDK_RECORD("TAKEOFF_DONE", "altitude_m=%.2f", target_alt_m);
    } else {
        SDK_LOG("takeoff(): altitude not confirmed within timeout");
        SDK_RECORD("TAKEOFF_FAILED", "error=%s", get_last_error_string());
    }
    return reached;
}

/* land()
 *   Hands landing to PX4's own AUTO.LAND, then confirms completion
 *   from EXTENDED_SYS_STATE (landed_state) and the disarm that PX4
 *   performs automatically once weight-on-wheels is detected. */
static inline bool land(void)
{
    SDK_REQUIRE_STATE(SDK_MASK_FLYING);
    SDK_LOG("land()");
    SDK_RECORD("LAND_START", "%s", "AUTO.LAND");
    if (!set_mode(MODE_LAND)) {
        SDK_RECORD("LAND_FAILED", "error=%s", get_last_error_string());
        return false;
    }

    if (!sdk_wait_until(sdk_check_landed, NULL, SDK_LAND_TIMEOUT_MS, 200)) {
        SDK_LOG("land(): landed_state never reached ON_GROUND");
        SDK_RECORD("LAND_FAILED", "error=%s", get_last_error_string());
        return false;
    }
    if (!sdk_wait_until(sdk_check_disarmed, NULL, SDK_LAND_TIMEOUT_MS, 200)) {
        SDK_LOG("land(): PX4 did not disarm after touchdown");
        SDK_RECORD("LAND_FAILED", "error=%s", get_last_error_string());
        return false;
    }
    SDK_LOG("land(): confirmed on ground and disarmed");
    sdk_set_state(SDK_READY);
    SDK_RECORD("LAND_DONE", "%s", "on ground and disarmed");
    return true;
}

/* land_at_rate(descent_rate_mps)
 *   Closed-loop Offboard descent at a caller-chosen rate instead of
 *   PX4's own landing speed. Disarms explicitly once landed_state
 *   confirms touchdown. */
static inline bool land_at_rate(float descent_rate_mps)
{
    SDK_REQUIRE_STATE(SDK_MASK_FLYING);
    float rate = fabsf(descent_rate_mps);
    SDK_LOG("land_at_rate(%.2f m/s)", rate);
    SDK_RECORD("LAND_START", "rate_mps=%.2f", rate);

    if (get_mode() != MODE_OFFBOARD && !sdk_enter_offboard_hold()) {
        SDK_RECORD("LAND_FAILED", "error=%s", get_last_error_string());
        return false;
    }
    sdk_set_state(SDK_LANDING);

    sdk_target_set_local_velocity(0.0f, 0.0f, rate, NAN); /* NED: +z is down */

    if (!sdk_wait_until(sdk_check_landed, NULL, SDK_LAND_TIMEOUT_MS, 200)) {
        SDK_LOG("land_at_rate(): landed_state never reached ON_GROUND");
        SDK_RECORD("LAND_FAILED", "error=%s", get_last_error_string());
        return false;
    }
    target_clear();
    if (!disarm()) {
        SDK_LOG("land_at_rate(): disarm failed after touchdown");
        SDK_RECORD("LAND_FAILED", "error=%s", get_last_error_string());
        return false;
    }
    SDK_LOG("land_at_rate(): confirmed on ground and disarmed");
    SDK_RECORD("LAND_DONE", "%s", "on ground and disarmed");
    return true;
}

/* rtl()
 *   Hands the return-to-launch flight entirely to PX4 (AUTO.RTL).
 *   Also usable as a recovery action straight out of EMERGENCY/ABORTED. */
static inline bool rtl(void)
{
    SDK_REQUIRE_STATE(SDK_MASK_FLYING_OR_RECOVERY);
    SDK_LOG("rtl()");
    SDK_RECORD("RTL_START", "%s", "AUTO.RTL");
    bool ok = set_mode(MODE_RETURN);
    SDK_RECORD(ok ? "RTL_DONE" : "RTL_FAILED", "%s", ok ? "mode confirmed" : get_last_error_string());
    return ok;
}

/* failsafe()
 *   The SDK's own top-level fail-safe response: return to launch.
 *   (PX4's own link-loss/battery/geofence failsafes run onboard the
 *   flight controller regardless of this function; this is a manual
 *   trigger mav.c or the watchdog can call.) */
static inline bool failsafe(void)
{
    SDK_LOG("failsafe() -> rtl()");
    return rtl();
}

/* hold_position()
 *   Hands position holding to PX4's own AUTO.LOITER. Also usable as a
 *   recovery action straight out of EMERGENCY/ABORTED. */
static inline bool hold_position(void)
{
    SDK_REQUIRE_STATE(SDK_MASK_FLYING_OR_RECOVERY);
    SDK_LOG("hold_position()");
    return set_mode(MODE_HOLD);
}

/* hover(ms)
 *   Holds position for ms milliseconds, kicking the watchdog. If the
 *   vehicle isn't in a state where holding makes sense, logs why and
 *   returns immediately instead of sleeping uselessly. */
static inline void hover(int ms)
{
    SDK_LOG("hover(%d ms)", ms);
    if (!hold_position()) {
        SDK_LOG("hover(): hold_position() failed (%s), not waiting", get_last_error_string());
        return;
    }
    int waited = 0;
    while (waited < ms && !g_request_stop) {
        kick_watchdog();
        sleep_ms(200);
        waited += 200;
    }
}

/* hover_forever()
 *   Holds position until stop_requested() (SIGINT/SIGTERM). */
static inline void hover_forever(void)
{
    SDK_LOG("hover_forever()");
    if (!hold_position()) {
        SDK_LOG("hover_forever(): hold_position() failed (%s), not waiting", get_last_error_string());
        return;
    }
    while (!g_request_stop) {
        kick_watchdog();
        sleep_ms(200);
    }
}

/* emergency_brake()
 *   Freezes the vehicle exactly where it is: cancels any in-progress
 *   waypoint or velocity command by overwriting the Offboard target
 *   with the current XYZ position and current heading, and keeps
 *   streaming it. Does not land, RTL, or disarm. Deliberately callable
 *   from almost any connected state - this is the one function that
 *   should never itself refuse to run. */
static inline bool emergency_brake(void)
{
    SDK_REQUIRE_STATE(SDK_MASK_CONNECTED);
    SDK_LOG("emergency_brake()");
    pthread_mutex_lock(&g_telem_lock);
    bool have_pos = g_telem.local_pos.valid;
    float x = g_telem.local_pos.x, y = g_telem.local_pos.y, z = g_telem.local_pos.z;
    float yaw = g_telem.yaw_rad;
    pthread_mutex_unlock(&g_telem_lock);

    if (!have_pos) {
        SDK_LOG("emergency_brake(): no position telemetry yet, cannot freeze");
        sdk_set_error(SDK_NO_TELEMETRY);
        SDK_RECORD("BRAKE_FAILED", "%s", "no position telemetry");
        return false;
    }
    sdk_target_set_local_position(x, y, z, yaw);
    sdk_set_state(SDK_EMERGENCY);
    sdk_set_error(SDK_ERR_ABORTED);
    SDK_RECORD("BRAKE", "frozen at x=%.2f y=%.2f z=%.2f", x, y, z);
    return true;
}

/* emergency_stop()
 *   Alias for emergency_brake(). The SDK deliberately never cuts
 *   motors in flight in response to a software condition - forcing a
 *   disarm mid-air is itself a crash. Freezing position is the safe
 *   maximal response; landing or disarming remain explicit, separate
 *   calls (land() / disarm()) so a human or mission layer decides. */
static inline bool emergency_stop(void)
{
    return emergency_brake();
}

/* Watchdog trip target, declared in sdk_threads.h. */
static inline void sdk_watchdog_trip(void)
{
    SDK_RECORD("WATCHDOG_TRIP", "%s", "mission loop unresponsive or link stale");
    emergency_brake();
}

#endif /* SDK_API_FLIGHT_H */
