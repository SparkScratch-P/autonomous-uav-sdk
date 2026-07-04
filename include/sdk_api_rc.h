/*
 * sdk_api_rc.h - The pilot's safety net: a dedicated RC channel
 * (SDK_RC_AUTONOMY_CHANNEL, default CH6) that must be held "on" for
 * autonomous commands to be allowed at all, plus a watch on PX4's
 * actual flight mode in case OFFBOARD is left unexpectedly.
 *
 * sdk_rc_override_check() is called every SDK_RC_MONITOR_POLL_MS by
 * the RC monitor thread (sdk_threads.h) - mav.c never calls it.
 */
#ifndef SDK_API_RC_H
#define SDK_API_RC_H

#include "sdk_state.h"
#include "sdk_log.h"
#include "sdk_error.h"
#include "sdk_fsm.h"
#include "sdk_recorder.h"
#include "sdk_target.h"
#include "sdk_api_core.h"

/* autonomy_enabled()
 *   True when the RC autonomy switch (or a manual enable_autonomy()
 *   call) currently permits autonomous commands. */
static inline bool autonomy_enabled(void)
{
    return g_autonomy_enabled;
}

/* pilot_override()
 *   True when the pilot currently has authority (autonomy switch off). */
static inline bool pilot_override(void)
{
    return !g_autonomy_enabled;
}

static inline void enable_autonomy(void)
{
    SDK_LOG("enable_autonomy()");
    g_autonomy_enabled = true;
}

static inline void disable_autonomy(void)
{
    SDK_LOG("disable_autonomy()");
    g_autonomy_enabled = false;
    target_clear();
}

/* sdk_rc_override_check()
 *   Definition of the function forward-declared in sdk_threads.h.
 *   Reads the autonomy channel and the current PX4 mode each tick:
 *     - switch goes OFF  -> clear targets, drop out of OFFBOARD so
 *                            the pilot's sticks regain authority.
 *     - PX4 leaves OFFBOARD on its own (pilot stick, failsafe, ...)
 *       while we thought we were flying autonomously -> treat it as
 *       an abort: clear targets, stop presenting as "offboard active". */
static inline void sdk_rc_override_check(void)
{
    pthread_mutex_lock(&g_telem_lock);
    bool have_rc = g_telem.rc.valid;
    uint16_t ch_us = g_telem.rc.raw_us[SDK_RC_AUTONOMY_CHANNEL - 1];
    uint32_t custom_mode = g_telem.custom_mode;
    pthread_mutex_unlock(&g_telem_lock);

    if (have_rc) {
        bool switch_on = ch_us >= SDK_RC_AUTONOMY_ON_US;
        if (switch_on != g_autonomy_enabled) {
            SDK_LOG("rc: autonomy switch -> %s", switch_on ? "ON" : "OFF");
            g_autonomy_enabled = switch_on;
            if (!switch_on) {
                target_clear();
                sdk_set_error(SDK_RC_OVERRIDE);
                sdk_set_state(SDK_ABORTED);
                SDK_RECORD("RC_OVERRIDE", "%s", "autonomy switch OFF, pilot has control");
                if (sdk_px4_to_flight_mode(custom_mode) == MODE_OFFBOARD) {
                    set_mode(MODE_POSITION);
                }
            }
        }
    }

    if (g_offboard_active && sdk_px4_to_flight_mode(custom_mode) != MODE_OFFBOARD) {
        SDK_LOG("rc: left OFFBOARD unexpectedly, aborting autonomous mission");
        g_offboard_active = false;
        target_clear();
        sdk_set_error(SDK_OFFBOARD_LOST);
        sdk_set_state(SDK_ABORTED);
        SDK_RECORD("OFFBOARD_LOST", "%s", "PX4 left OFFBOARD mode on its own");
    }
}

/* monitor_rc_override()
 *   Runs one override check synchronously. The background thread
 *   already does this continuously; this is exposed for mission code
 *   that wants to react to a fresh check at a specific point. */
static inline void monitor_rc_override(void)
{
    sdk_rc_override_check();
}

#endif /* SDK_API_RC_H */
