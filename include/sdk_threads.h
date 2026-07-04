/*
 * sdk_threads.h - The five background threads that keep the link
 * alive, keep telemetry fresh, keep Offboard fed, and keep the
 * mission honest.
 *
 * mav.c never sees any of this - start() launches them, stop() joins
 * them, that is the entire interface mav.c needs.
 */
#ifndef SDK_THREADS_H
#define SDK_THREADS_H

#include <stddef.h>
#include "sdk_state.h"
#include "sdk_log.h"
#include "sdk_error.h"
#include "sdk_time_util.h"
#include "sdk_serial.h"
#include "sdk_mavlink_send.h"
#include "sdk_mavlink_recv.h"

/* --- Heartbeat thread ------------------------------------------------
 * PX4 requires a companion-computer heartbeat to accept Offboard mode
 * and to keep treating us as a valid MAVLink peer. */
static inline void *sdk_heartbeat_thread_fn(void *arg)
{
    (void)arg;
    g_heartbeat_thread_alive = true;
    while (g_threads_running) {
        sdk_send_heartbeat();
        sleep_ms(SDK_HEARTBEAT_PERIOD_MS);
    }
    g_heartbeat_thread_alive = false;
    return NULL;
}

/* --- Telemetry reader thread ------------------------------------------
 * Reads raw bytes, feeds them through the MAVLink parser one byte at a
 * time, and hands every complete message to sdk_handle_message(). */
static inline void *sdk_telemetry_thread_fn(void *arg)
{
    (void)arg;
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t byte;

    g_telemetry_thread_alive = true;
    while (g_threads_running) {
        pthread_mutex_lock(&g_serial_lock);
        int fd = g_fd;
        pthread_mutex_unlock(&g_serial_lock);
        if (fd < 0) { sleep_ms(50); continue; }

        ssize_t n = read(fd, &byte, 1);
        if (n <= 0) { sleep_ms(2); continue; }

        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
            sdk_handle_message(&msg);
        }
    }
    g_telemetry_thread_alive = false;
    return NULL;
}

/* --- Offboard streaming thread -----------------------------------------
 * Streams whatever g_target currently holds at SDK_OFFBOARD_STREAM_HZ.
 * This must never stop while Offboard is engaged - PX4 falls back out
 * of Offboard if setpoints stop arriving. When there is no target set
 * (TARGET_NONE) it still streams the current position, so the vehicle
 * simply holds still instead of PX4 losing the stream entirely. */
static inline void *sdk_offboard_thread_fn(void *arg)
{
    (void)arg;
    g_offboard_thread_alive = true;
    while (g_threads_running) {
        pthread_mutex_lock(&g_target_lock);
        target_kind_t kind = g_target.kind;
        float x = g_target.x, y = g_target.y, z = g_target.z;
        double lat = g_target.lat, lon = g_target.lon;
        float alt_m = g_target.alt_m;
        float yaw = g_target.has_yaw ? g_target.yaw_rad : NAN;
        pthread_mutex_unlock(&g_target_lock);

        switch (kind) {
        case TARGET_LOCAL_POSITION:
            sdk_send_setpoint_local_position(x, y, z, yaw);
            break;
        case TARGET_LOCAL_VELOCITY:
            sdk_send_setpoint_local_velocity(x, y, z, yaw);
            break;
        case TARGET_GLOBAL_POSITION:
            sdk_send_setpoint_global_position(lat, lon, alt_m, yaw);
            break;
        case TARGET_YAW_RATE:
            sdk_send_setpoint_yaw_rate(x); /* rate stashed in .x, see sdk_target_set_yaw_rate() */
            break;
        case TARGET_NONE:
        default: {
            /* Hold current position so the stream never goes silent. */
            pthread_mutex_lock(&g_telem_lock);
            float cx = g_telem.local_pos.x, cy = g_telem.local_pos.y, cz = g_telem.local_pos.z;
            bool have_pos = g_telem.local_pos.valid;
            pthread_mutex_unlock(&g_telem_lock);
            if (have_pos) sdk_send_setpoint_local_position(cx, cy, cz, NAN);
            break;
        }
        }
        sleep_ms(SDK_OFFBOARD_STREAM_MS);
    }
    g_offboard_thread_alive = false;
    return NULL;
}

/* --- Watchdog thread -----------------------------------------------
 * If the mission logic in mav.c stops kicking the watchdog (hung,
 * crashed loop, whatever) for longer than SDK_WATCHDOG_TIMEOUT_MS, or
 * PX4's own HEARTBEAT goes stale, freeze in place exactly like
 * emergency_brake() would - never keep flying blind. */
static inline void sdk_watchdog_trip(void); /* implemented in sdk_api_flight.h, declared here */

static inline void *sdk_watchdog_thread_fn(void *arg)
{
    (void)arg;
    g_watchdog_thread_alive = true;
    while (g_threads_running) {
        pthread_mutex_lock(&g_kick_lock);
        struct timespec last = g_last_kick;
        pthread_mutex_unlock(&g_kick_lock);

        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        long age_ms = (now.tv_sec - last.tv_sec) * 1000L +
                      (now.tv_nsec - last.tv_nsec) / 1000000L;

        if (age_ms > SDK_WATCHDOG_TIMEOUT_MS && g_offboard_active) {
            SDK_LOG("watchdog: mission loop unresponsive for %ld ms, freezing position", age_ms);
            sdk_watchdog_trip();
        }

        pthread_mutex_lock(&g_telem_lock);
        bool have_hb = g_telem.have_heartbeat;
        struct timespec last_hb = g_telem.last_heartbeat;
        pthread_mutex_unlock(&g_telem_lock);
        if (have_hb) {
            long hb_age_ms = (now.tv_sec - last_hb.tv_sec) * 1000L +
                              (now.tv_nsec - last_hb.tv_nsec) / 1000000L;
            if (hb_age_ms > 3 * SDK_HEARTBEAT_PERIOD_MS && g_offboard_active) {
                SDK_LOG("watchdog: PX4 heartbeat stale for %ld ms, freezing position", hb_age_ms);
                sdk_set_error(SDK_CONNECTION_LOST);
                sdk_watchdog_trip();
            }
        }

        sleep_ms(SDK_WATCHDOG_POLL_MS);
    }
    g_watchdog_thread_alive = false;
    return NULL;
}

/* --- RC override monitor thread --------------------------------------
 * Watches the autonomy-enable RC channel and the current PX4 mode.
 * Implemented against sdk_api_rc.h's sdk_rc_override_check(), declared
 * here to avoid a header ordering dependency. */
static inline void sdk_rc_override_check(void); /* implemented in sdk_api_rc.h */

static inline void *sdk_rc_monitor_thread_fn(void *arg)
{
    (void)arg;
    g_rc_monitor_thread_alive = true;
    while (g_threads_running) {
        sdk_rc_override_check();
        sleep_ms(SDK_RC_MONITOR_POLL_MS);
    }
    g_rc_monitor_thread_alive = false;
    return NULL;
}

/* --- Lifecycle --------------------------------------------------------*/

typedef struct {
    pthread_t *handle;
    void *(*fn)(void *);
    const volatile bool *alive;
    const char *name;
} sdk_thread_spec_t;

static inline size_t sdk_thread_specs(sdk_thread_spec_t out[5])
{
    out[0] = (sdk_thread_spec_t){ &g_heartbeat_thread,  sdk_heartbeat_thread_fn,  &g_heartbeat_thread_alive,  "heartbeat"  };
    out[1] = (sdk_thread_spec_t){ &g_telemetry_thread,  sdk_telemetry_thread_fn,  &g_telemetry_thread_alive,  "telemetry"  };
    out[2] = (sdk_thread_spec_t){ &g_offboard_thread,   sdk_offboard_thread_fn,   &g_offboard_thread_alive,   "offboard"   };
    out[3] = (sdk_thread_spec_t){ &g_watchdog_thread,   sdk_watchdog_thread_fn,   &g_watchdog_thread_alive,   "watchdog"   };
    out[4] = (sdk_thread_spec_t){ &g_rc_monitor_thread, sdk_rc_monitor_thread_fn, &g_rc_monitor_thread_alive, "rc_monitor" };
    return 5;
}

/* sdk_threads_all_alive()
 *   True once every background thread has actually entered its main
 *   loop (not merely been handed to pthread_create() successfully). */
static inline bool sdk_threads_all_alive(void)
{
    sdk_thread_spec_t specs[5];
    size_t n = sdk_thread_specs(specs);
    for (size_t i = 0; i < n; i++) {
        if (!*specs[i].alive) return false;
    }
    return true;
}

/* sdk_threads_start()
 *   Starts all five threads. If any pthread_create() call fails, every
 *   thread already started is signaled to exit and joined, and this
 *   returns false - the SDK is never left half-initialized. */
static inline bool sdk_threads_start(void)
{
    g_heartbeat_thread_alive  = false;
    g_telemetry_thread_alive  = false;
    g_offboard_thread_alive   = false;
    g_watchdog_thread_alive   = false;
    g_rc_monitor_thread_alive = false;
    g_threads_running = true;

    sdk_thread_spec_t specs[5];
    size_t n = sdk_thread_specs(specs);
    size_t started = 0;

    for (started = 0; started < n; started++) {
        int rc = pthread_create(specs[started].handle, NULL, specs[started].fn, NULL);
        if (rc != 0) {
            SDK_LOG("threads: failed to start %s thread (error %d)", specs[started].name, rc);
            break;
        }
    }

    if (started < n) {
        SDK_LOG("threads: rolling back %zu already-started thread(s)", started);
        g_threads_running = false; /* tells every started thread to exit its loop */
        for (size_t i = 0; i < started; i++) {
            pthread_join(*specs[i].handle, NULL);
        }
        return false;
    }
    return true;
}

static inline void sdk_threads_stop(void)
{
    g_threads_running = false;
    sdk_thread_spec_t specs[5];
    size_t n = sdk_thread_specs(specs);
    for (size_t i = 0; i < n; i++) {
        pthread_join(*specs[i].handle, NULL);
    }
}

#endif /* SDK_THREADS_H */
