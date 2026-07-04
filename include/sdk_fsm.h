/*
 * sdk_fsm.h - The internal SDK lifecycle state machine.
 *
 * This is what every public API function checks before doing
 * anything: calling arm() while DISCONNECTED, or gps_go() while
 * LANDING, fails immediately with SDK_BAD_STATE instead of sending
 * MAVLink traffic into a situation that doesn't make sense.
 *
 * Mission code never touches this directly - get_sdk_state() /
 * get_sdk_state_name() exist for introspection/logging only. No
 * existing mission code needs to change: every legal call sequence
 * the public API already documented (start -> arm -> takeoff ->
 * gps_go -> ... -> land -> stop) walks a legal path through this
 * machine by construction.
 */
#ifndef SDK_FSM_H
#define SDK_FSM_H

#include <pthread.h>
#include <stdint.h>
#include "sdk_log.h"

typedef enum {
    SDK_DISCONNECTED = 0, /* before start() / after a failed start() */
    SDK_CONNECTING,         /* inside start(), link opening / readiness not yet confirmed */
    SDK_READY,               /* linked to PX4, disarmed, telemetry flowing */
    SDK_ARMED,                /* armed, on the ground, not yet in Offboard */
    SDK_OFFBOARD,               /* in OFFBOARD, holding position, no active maneuver */
    SDK_MISSION,                 /* a maneuver is actively in progress (takeoff/nav/yaw/velocity) */
    SDK_HOLD,                     /* arrived / holding position */
    SDK_LANDING,                   /* land()/land_at_rate() in progress */
    SDK_RTL,                         /* rtl()/failsafe() in progress */
    SDK_EMERGENCY,                     /* emergency_brake(), watchdog trip, or link failure */
    SDK_ABORTED,                        /* RC override cutoff, Offboard lost, or abort_mission() */
    SDK_STOPPED,                          /* after stop() - fully reusable, call start() again */
} sdk_state_t;

#define SDK_BIT(s) (1u << (uint32_t)(s))

/* Convenience masks used by the guards in the sdk_api_*.h files. */
#define SDK_MASK_FLYING \
    (SDK_BIT(SDK_ARMED) | SDK_BIT(SDK_OFFBOARD) | SDK_BIT(SDK_MISSION) | SDK_BIT(SDK_HOLD) | \
     SDK_BIT(SDK_LANDING) | SDK_BIT(SDK_RTL))
#define SDK_MASK_FLYING_OR_RECOVERY \
    (SDK_MASK_FLYING | SDK_BIT(SDK_EMERGENCY) | SDK_BIT(SDK_ABORTED))
#define SDK_MASK_CONNECTED \
    (~(SDK_BIT(SDK_DISCONNECTED) | SDK_BIT(SDK_CONNECTING) | SDK_BIT(SDK_STOPPED)))

static pthread_mutex_t g_fsm_lock = PTHREAD_MUTEX_INITIALIZER;
static sdk_state_t g_sdk_state = SDK_DISCONNECTED;

static inline const char *sdk_state_name(sdk_state_t s)
{
    switch (s) {
        case SDK_DISCONNECTED: return "DISCONNECTED";
        case SDK_CONNECTING:   return "CONNECTING";
        case SDK_READY:        return "READY";
        case SDK_ARMED:        return "ARMED";
        case SDK_OFFBOARD:     return "OFFBOARD";
        case SDK_MISSION:      return "MISSION";
        case SDK_HOLD:         return "HOLD";
        case SDK_LANDING:      return "LANDING";
        case SDK_RTL:          return "RTL";
        case SDK_EMERGENCY:    return "EMERGENCY";
        case SDK_ABORTED:      return "ABORTED";
        case SDK_STOPPED:      return "STOPPED";
        default:                return "UNKNOWN";
    }
}

/* get_sdk_state() / get_sdk_state_name()
 *   Read-only introspection - what the SDK believes is happening
 *   right now. Not required for any mission script to run. */
static inline sdk_state_t get_sdk_state(void)
{
    pthread_mutex_lock(&g_fsm_lock);
    sdk_state_t v = g_sdk_state;
    pthread_mutex_unlock(&g_fsm_lock);
    return v;
}

static inline const char *get_sdk_state_name(void)
{
    return sdk_state_name(get_sdk_state());
}

/* sdk_set_state()
 *   Called automatically by the SDK on every successful transition.
 *   Logs every change - a free timeline of the mission alongside the
 *   flight recorder's persisted trace. */
static inline void sdk_set_state(sdk_state_t new_state)
{
    pthread_mutex_lock(&g_fsm_lock);
    sdk_state_t old = g_sdk_state;
    g_sdk_state = new_state;
    pthread_mutex_unlock(&g_fsm_lock);
    if (old != new_state) {
        SDK_LOG("state: %s -> %s", sdk_state_name(old), sdk_state_name(new_state));
    }
}

#endif /* SDK_FSM_H */
