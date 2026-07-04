/*
 * sdk_api_core.h - start/stop, arming, and flight-mode control.
 */
#ifndef SDK_API_CORE_H
#define SDK_API_CORE_H

#include "sdk_state.h"
#include "sdk_log.h"
#include "sdk_error.h"
#include "sdk_fsm.h"
#include "sdk_recorder.h"
#include "sdk_time_util.h"
#include "sdk_serial.h"
#include "sdk_mavlink_send.h"
#include "sdk_commands.h"
#include "sdk_threads.h"
#include "sdk_target.h"

static inline uint32_t sdk_flight_mode_to_px4(flight_mode_t mode)
{
    switch (mode) {
        case MODE_MANUAL:     return PX4_CUSTOM_MODE(PX4_MAIN_MANUAL, 0);
        case MODE_STABILIZED: return PX4_CUSTOM_MODE(PX4_MAIN_STABILIZED, 0);
        case MODE_ALTITUDE:   return PX4_CUSTOM_MODE(PX4_MAIN_ALTCTL, 0);
        case MODE_POSITION:   return PX4_CUSTOM_MODE(PX4_MAIN_POSCTL, 0);
        case MODE_HOLD:       return PX4_CUSTOM_MODE(PX4_MAIN_AUTO, PX4_SUB_AUTO_LOITER);
        case MODE_MISSION:    return PX4_CUSTOM_MODE(PX4_MAIN_AUTO, PX4_SUB_AUTO_MISSION);
        case MODE_OFFBOARD:   return PX4_CUSTOM_MODE(PX4_MAIN_OFFBOARD, 0);
        case MODE_RETURN:     return PX4_CUSTOM_MODE(PX4_MAIN_AUTO, PX4_SUB_AUTO_RTL);
        case MODE_LAND:       return PX4_CUSTOM_MODE(PX4_MAIN_AUTO, PX4_SUB_AUTO_LAND);
        default:              return 0;
    }
}

static inline flight_mode_t sdk_px4_to_flight_mode(uint32_t custom_mode)
{
    uint8_t main = (uint8_t)((custom_mode >> 16) & 0xFF);
    uint8_t sub  = (uint8_t)((custom_mode >> 24) & 0xFF);
    switch (main) {
        case PX4_MAIN_MANUAL:     return MODE_MANUAL;
        case PX4_MAIN_STABILIZED: return MODE_STABILIZED;
        case PX4_MAIN_ALTCTL:     return MODE_ALTITUDE;
        case PX4_MAIN_POSCTL:     return MODE_POSITION;
        case PX4_MAIN_OFFBOARD:   return MODE_OFFBOARD;
        case PX4_MAIN_AUTO:
            switch (sub) {
                case PX4_SUB_AUTO_LOITER:  return MODE_HOLD;
                case PX4_SUB_AUTO_MISSION: return MODE_MISSION;
                case PX4_SUB_AUTO_RTL:     return MODE_RETURN;
                case PX4_SUB_AUTO_LAND:    return MODE_LAND;
                default:                   return MODE_UNKNOWN;
            }
        default: return MODE_UNKNOWN;
    }
}

/* sdk_startup_abort()
 *   Shared cleanup for every start() failure path: stop any threads
 *   that did come up, close the port, drop back to DISCONNECTED. */
static inline void sdk_startup_abort(bool threads_were_started)
{
    if (threads_were_started) sdk_threads_stop();
    sdk_serial_close(g_fd);
    g_fd = -1;
    sdk_set_state(SDK_DISCONNECTED);
}

/* start(port, baud)
 *   Opens the serial link, launches every background thread, and
 *   blocks until the SDK is genuinely ready to fly:
 *     - HEARTBEAT received
 *     - target system/component discovered
 *     - ATTITUDE received
 *     - LOCAL_POSITION_NED valid
 *     - GLOBAL_POSITION_INT valid
 *     - every background thread confirmed running
 *   Returns false, with the SDK left fully reusable, if the port
 *   can't be opened, any thread fails to start, or any condition
 *   above isn't met within SDK_STARTUP_READY_TIMEOUT_MS. */
static inline bool start(const char *port, int baud)
{
    sdk_state_t cur = get_sdk_state();
    if (cur != SDK_DISCONNECTED && cur != SDK_STOPPED) {
        SDK_LOG("start(): already connected (state=%s)", sdk_state_name(cur));
        sdk_set_error(SDK_BAD_STATE);
        return false;
    }

    clock_gettime(CLOCK_MONOTONIC, &g_t0);
    kick_watchdog();
    pthread_cond_init(&g_telem.ack_cond, NULL);
    sdk_set_state(SDK_CONNECTING);

    char recorder_path[64];
    snprintf(recorder_path, sizeof(recorder_path), "flight_%ld.csv", (long)time(NULL));
    sdk_recorder_open(recorder_path);
    SDK_RECORD("START", "port=%s baud=%d", port, baud);

    SDK_LOG("start(%s, %d)", port, baud);
    g_fd = sdk_serial_open(port, baud);
    if (g_fd < 0) {
        sdk_set_error(SDK_CONNECTION_LOST);
        SDK_RECORD("START_FAILED", "could not open %s", port);
        sdk_startup_abort(false);
        return false;
    }

    if (!sdk_threads_start()) {
        sdk_set_error(SDK_BAD_STATE);
        SDK_RECORD("START_FAILED", "%s", "one or more background threads failed to start");
        sdk_startup_abort(false); /* sdk_threads_start() already rolled back what it created */
        return false;
    }

    int waited = 0;
    while (waited < SDK_STARTUP_READY_TIMEOUT_MS) {
        if (sdk_check_startup_ready(NULL) && sdk_threads_all_alive()) {
            SDK_LOG("start(): all startup conditions satisfied");
            sdk_set_state(SDK_READY);
            sdk_set_error(SDK_OK);
            SDK_RECORD("READY", "%s", "heartbeat+target+attitude+position+threads confirmed");
            return true;
        }
        sleep_ms(100);
        waited += 100;
    }

    SDK_LOG("start(): startup conditions not satisfied within %d ms", SDK_STARTUP_READY_TIMEOUT_MS);
    sdk_set_error(SDK_TIMEOUT);
    SDK_RECORD("START_FAILED", "%s", "startup readiness timeout");
    sdk_startup_abort(true);
    return false;
}

/* stop()
 *   Always allowed, from any state. Joins every thread, clears the
 *   Offboard target, disables Offboard streaming, closes the serial
 *   port, and resets every piece of session state (telemetry cache
 *   validity flags, discovered target sysid/compid, last error) so a
 *   subsequent start() begins from a genuinely clean slate rather than
 *   racing stale data left over from this session. */
static inline void stop(void)
{
    SDK_LOG("stop(): shutting down");
    SDK_RECORD("STOP", "%s", "shutting down");

    g_offboard_active = false;
    target_clear();
    sdk_threads_stop();
    sdk_serial_close(g_fd);
    g_fd = -1;

    g_target_known = false;
    pthread_mutex_lock(&g_telem_lock);
    g_telem.have_heartbeat   = false;
    g_telem.have_attitude    = false;
    g_telem.local_pos.valid  = false;
    g_telem.have_global_pos  = false;
    g_telem.have_landed_state = false;
    g_telem.have_odometry    = false;
    g_telem.ack_pending      = false;
    pthread_mutex_unlock(&g_telem_lock);

    sdk_set_error(SDK_OK);
    sdk_set_state(SDK_STOPPED);
    SDK_LOG("stop(): done, SDK is ready for another start()");
    sdk_recorder_close();
}

/* restart()
 *   There is no portable way to recover a serial device path from an
 *   open fd, so a bare restart() cannot know what port to reopen.
 *   Callers should call stop() followed by start(port, baud) with the
 *   same arguments instead. This function exists to satisfy the public
 *   API and always returns false. */
static inline bool restart(void)
{
    SDK_LOG("restart(): call stop() then start(port, baud) explicitly instead.");
    return false;
}

static inline bool stop_requested(void)
{
    return g_request_stop != 0;
}

/* --- Arming ----------------------------------------------------------*/

static inline bool arm(void)
{
    if (get_sdk_state() == SDK_ARMED) return true; /* already armed: idempotent */
    SDK_REQUIRE_STATE(SDK_BIT(SDK_READY));
    SDK_LOG("arm()");

    bool ok = sdk_send_command_and_wait(MAV_CMD_COMPONENT_ARM_DISARM,
                                         1, 0, 0, 0, 0, 0, 0,
                                         SDK_COMMAND_ACK_TIMEOUT_MS);
    if (!ok) { SDK_RECORD("ARM_FAILED", "error=%s", get_last_error_string()); return false; }

    bool armed = sdk_wait_until(sdk_check_armed, NULL, SDK_ARM_TIMEOUT_MS, 100);
    if (armed) {
        sdk_set_state(SDK_ARMED);
        SDK_RECORD("ARMED", "%s", "confirmed");
    } else {
        SDK_RECORD("ARM_FAILED", "error=%s", get_last_error_string());
    }
    return armed;
}

static inline bool disarm(void)
{
    if (get_sdk_state() == SDK_READY) return true; /* already disarmed: idempotent */
    SDK_REQUIRE_STATE(SDK_MASK_FLYING_OR_RECOVERY);
    SDK_LOG("disarm()");

    bool ok = sdk_send_command_and_wait(MAV_CMD_COMPONENT_ARM_DISARM,
                                         0, 0, 0, 0, 0, 0, 0,
                                         SDK_COMMAND_ACK_TIMEOUT_MS);
    if (!ok) { SDK_RECORD("DISARM_FAILED", "error=%s", get_last_error_string()); return false; }

    bool disarmed = sdk_wait_until(sdk_check_disarmed, NULL, SDK_ARM_TIMEOUT_MS, 100);
    if (disarmed) {
        sdk_set_state(SDK_READY);
        SDK_RECORD("DISARMED", "%s", "confirmed");
    } else {
        SDK_RECORD("DISARM_FAILED", "error=%s", get_last_error_string());
    }
    return disarmed;
}

static inline bool is_armed(void)
{
    pthread_mutex_lock(&g_telem_lock);
    bool v = g_telem.armed;
    pthread_mutex_unlock(&g_telem_lock);
    return v;
}

static inline bool is_landed(void)
{
    pthread_mutex_lock(&g_telem_lock);
    bool v = g_telem.have_landed_state && g_telem.landed_state == MAV_LANDED_STATE_ON_GROUND;
    pthread_mutex_unlock(&g_telem_lock);
    return v;
}

static inline bool is_flying(void)
{
    return is_armed() && !is_landed();
}

/* --- Flight mode -------------------------------------------------------*/

/* sdk_state_for_mode()
 *   The coarse SDK state a bare set_mode() switch settles into. More
 *   specific calls (takeoff(), gps_go(), ...) override this with a
 *   more precise state right after switching to OFFBOARD themselves. */
static inline sdk_state_t sdk_state_for_mode(flight_mode_t mode)
{
    switch (mode) {
        case MODE_OFFBOARD: return SDK_OFFBOARD;
        case MODE_HOLD:     return SDK_HOLD;
        case MODE_LAND:     return SDK_LANDING;
        case MODE_RETURN:   return SDK_RTL;
        default:            return get_sdk_state(); /* MANUAL/STABILIZED/etc: pilot's business, leave ours alone */
    }
}

static inline bool set_mode(flight_mode_t mode)
{
    SDK_REQUIRE_STATE(SDK_MASK_CONNECTED);
    SDK_LOG("set_mode(%d)", (int)mode);
    uint32_t px4_mode = sdk_flight_mode_to_px4(mode);
    sdk_send_set_mode(px4_mode);
    bool reached = sdk_wait_until(sdk_check_custom_mode, &px4_mode, SDK_MODE_TIMEOUT_MS, 100);
    g_offboard_active = (mode == MODE_OFFBOARD) && reached;

    if (reached) {
        sdk_set_state(sdk_state_for_mode(mode));
        SDK_RECORD("SET_MODE", "mode=%d result=OK", (int)mode);
    } else {
        SDK_RECORD("SET_MODE_FAILED", "mode=%d error=%s", (int)mode, get_last_error_string());
    }
    return reached;
}

static inline flight_mode_t get_mode(void)
{
    pthread_mutex_lock(&g_telem_lock);
    uint32_t cm = g_telem.custom_mode;
    pthread_mutex_unlock(&g_telem_lock);
    return sdk_px4_to_flight_mode(cm);
}

static inline bool wait_until_mode(flight_mode_t mode, int timeout_ms)
{
    uint32_t px4_mode = sdk_flight_mode_to_px4(mode);
    return sdk_wait_until(sdk_check_custom_mode, &px4_mode, timeout_ms, 100);
}

#endif /* SDK_API_CORE_H */
