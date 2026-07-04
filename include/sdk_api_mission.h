/*
 * sdk_api_mission.h - Coarse mission control for the PX4 AUTO.MISSION
 * flight mode, plus wait() - the watchdog-safe sleep every blocking
 * call in this SDK is built on.
 */
#ifndef SDK_API_MISSION_H
#define SDK_API_MISSION_H

#include "sdk_state.h"
#include "sdk_log.h"
#include "sdk_time_util.h"
#include "sdk_commands.h"
#include "sdk_api_core.h"
#include "sdk_api_flight.h"

/* pause_mission()
 *   Pauses a running PX4 mission in place (MAV_CMD_DO_PAUSE_CONTINUE). */
static inline bool pause_mission(void)
{
    SDK_REQUIRE_STATE(SDK_MASK_FLYING);
    SDK_LOG("pause_mission()");
    return sdk_send_command_and_wait(MAV_CMD_DO_PAUSE_CONTINUE, 0, 0, 0, 0, 0, 0, 0,
                                      SDK_COMMAND_ACK_TIMEOUT_MS);
}

/* resume_mission()
 *   Resumes a paused PX4 mission from where it left off. */
static inline bool resume_mission(void)
{
    SDK_REQUIRE_STATE(SDK_MASK_FLYING);
    SDK_LOG("resume_mission()");
    return sdk_send_command_and_wait(MAV_CMD_DO_PAUSE_CONTINUE, 1, 0, 0, 0, 0, 0, 0,
                                      SDK_COMMAND_ACK_TIMEOUT_MS);
}

/* abort_mission()
 *   Leaves AUTO.MISSION and holds position - the mission sequence
 *   number stops advancing, but the vehicle keeps flying itself. */
static inline bool abort_mission(void)
{
    SDK_LOG("abort_mission()");
    return hold_position();
}

/* mission_running()
 *   True while armed and in PX4's AUTO.MISSION flight mode. */
static inline bool mission_running(void)
{
    return is_armed() && get_mode() == MODE_MISSION;
}

/* wait(ms)
 *   Blocks for ms milliseconds without touching the flight mode or
 *   Offboard target - whatever the vehicle was doing keeps happening.
 *   Kicks the watchdog throughout, same as every other blocking call. */
static inline void wait(int ms)
{
    int waited = 0;
    while (waited < ms && !g_request_stop) {
        kick_watchdog();
        sleep_ms(200);
        waited += 200;
    }
}

#endif /* SDK_API_MISSION_H */
