/*
 * sdk_commands.h - Turns "send a command" into "send it and prove it
 * actually happened", using the telemetry cache and COMMAND_ACK.
 *
 * This is the file that makes the whole SDK closed-loop: nothing in
 * the API layer estimates success from elapsed time. It also owns
 * g_command_lock, which serializes every ACK-waiting COMMAND_LONG
 * transaction so at most one is ever outstanding at a time - see the
 * comment on sdk_send_command_and_wait() below.
 */
#ifndef SDK_COMMANDS_H
#define SDK_COMMANDS_H

#include <math.h>
#include "sdk_state.h"
#include "sdk_log.h"
#include "sdk_error.h"
#include "sdk_fsm.h"
#include "sdk_recorder.h"
#include "sdk_time_util.h"
#include "sdk_mavlink_send.h"
#include "sdk_types.h"

/* SDK_REQUIRE_STATE(mask)
 *   Guard placed at the top of every public function that actually
 *   commands the vehicle. If the current lifecycle state isn't in
 *   `mask` (an OR of SDK_BIT(SDK_xxx) values, see sdk_fsm.h), the call
 *   fails immediately: no MAVLink traffic is sent, SDK_BAD_STATE is
 *   recorded as the error, and the illegal call is logged. Every
 *   legal call sequence the public API already documents walks a
 *   legal path through the masks used below, so no mission code needs
 *   to change. */
#define SDK_REQUIRE_STATE(mask) \
    do { \
        sdk_state_t __cur = get_sdk_state(); \
        if ((SDK_BIT(__cur) & (mask)) == 0) { \
            SDK_LOG("%s: illegal call in state %s", __func__, sdk_state_name(__cur)); \
            sdk_set_error(SDK_BAD_STATE); \
            SDK_RECORD("BAD_STATE", "fn=%s state=%s", __func__, sdk_state_name(__cur)); \
            return false; \
        } \
    } while (0)

/* g_command_lock
 *   Held for the full duration of every ACK-waiting COMMAND_LONG
 *   transaction (send through wait-for-ACK). This guarantees at most
 *   one such transaction is outstanding at any time, regardless of
 *   how many threads call into the SDK concurrently (mission thread,
 *   RC monitor thread, watchdog thread), which is what makes matching
 *   an incoming COMMAND_ACK to "the" in-flight command correct: there
 *   is only ever one candidate. (MAVLink's COMMAND_ACK does not carry
 *   a per-transaction id, so this serialization - not clever matching
 *   - is what actually prevents cross-thread ACK mismatches; a late
 *   ACK for a timed-out command could in principle still be mistaken
 *   for the ACK of an immediately-following retry of the *same*
 *   command id, which is an inherent MAVLink protocol limitation no
 *   SDK-side logic can fully close.) */
static pthread_mutex_t g_command_lock = PTHREAD_MUTEX_INITIALIZER;

/* sdk_send_command_and_wait()
 *   Sends a COMMAND_LONG and blocks until COMMAND_ACK for that command
 *   arrives (MAV_RESULT_ACCEPTED), or timeout_ms elapses.
 *   Returns true only on MAV_RESULT_ACCEPTED. */
static inline bool sdk_send_command_and_wait(uint16_t command,
                                              float p1, float p2, float p3, float p4,
                                              float p5, float p6, float p7,
                                              int timeout_ms)
{
    pthread_mutex_lock(&g_command_lock);

    pthread_mutex_lock(&g_telem_lock);
    g_telem.ack_pending = true;
    g_telem.ack_command = 0;
    pthread_mutex_unlock(&g_telem_lock);

    sdk_send_command_long(command, p1, p2, p3, p4, p5, p6, p7);

    struct timespec deadline;
    clock_gettime(CLOCK_REALTIME, &deadline);
    deadline.tv_sec  += timeout_ms / 1000;
    deadline.tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
    if (deadline.tv_nsec >= 1000000000L) { deadline.tv_sec++; deadline.tv_nsec -= 1000000000L; }

    pthread_mutex_lock(&g_telem_lock);
    while (g_telem.ack_pending || g_telem.ack_command != command) {
        int rc = pthread_cond_timedwait(&g_telem.ack_cond, &g_telem_lock, &deadline);
        if (rc != 0) break; /* timeout */
        if (g_telem.ack_command == command) break;
    }
    bool got_it   = (g_telem.ack_command == command) && !g_telem.ack_pending;
    uint8_t result = g_telem.ack_result;
    pthread_mutex_unlock(&g_telem_lock);

    bool ok;
    if (!got_it) {
        SDK_LOG("command %u: timed out waiting for ACK", command);
        sdk_set_error(SDK_TIMEOUT);
        ok = false;
    } else if (result != MAV_RESULT_ACCEPTED) {
        SDK_LOG("command %u: rejected (result=%u)", command, result);
        sdk_set_error(SDK_ACK_REJECTED);
        ok = false;
    } else {
        sdk_set_error(SDK_OK);
        ok = true;
    }

    pthread_mutex_unlock(&g_command_lock);
    return ok;
}

/* sdk_wait_until()
 *   Generic closed-loop poll: calls check() every poll_ms, kicking the
 *   watchdog each time, until it returns true or timeout_ms elapses.
 *   check_arg is passed through untouched (e.g. a target altitude). */
typedef bool (*sdk_condition_fn)(const void *arg);

static inline bool sdk_wait_until(sdk_condition_fn check, const void *arg,
                                   int timeout_ms, int poll_ms)
{
    int waited = 0;
    while (waited <= timeout_ms) {
        if (g_request_stop) {
            sdk_set_error(SDK_ERR_ABORTED);
            return false;
        }
        kick_watchdog();
        if (check(arg)) {
            sdk_set_error(SDK_OK);
            return true;
        }
        sleep_ms(poll_ms);
        waited += poll_ms;
    }
    sdk_set_error(SDK_TIMEOUT);
    return false;
}

/* --- Ready-made condition checks used across the API layer ---------- */

static inline bool sdk_check_armed(const void *arg)
{
    (void)arg;
    pthread_mutex_lock(&g_telem_lock);
    bool armed = g_telem.armed;
    pthread_mutex_unlock(&g_telem_lock);
    return armed;
}

static inline bool sdk_check_disarmed(const void *arg)
{
    (void)arg;
    pthread_mutex_lock(&g_telem_lock);
    bool armed = g_telem.armed;
    pthread_mutex_unlock(&g_telem_lock);
    return !armed;
}

static inline bool sdk_check_custom_mode(const void *arg)
{
    uint32_t want = *(const uint32_t *)arg;
    pthread_mutex_lock(&g_telem_lock);
    uint32_t have = g_telem.custom_mode;
    pthread_mutex_unlock(&g_telem_lock);
    return have == want;
}

static inline bool sdk_check_alt_rel(const void *arg)
{
    float target_m = *(const float *)arg;
    pthread_mutex_lock(&g_telem_lock);
    float alt = g_telem.alt_rel_m;
    pthread_mutex_unlock(&g_telem_lock);
    return fabsf(alt - target_m) <= SDK_ALT_TOLERANCE_M;
}

static inline bool sdk_check_landed(const void *arg)
{
    (void)arg;
    pthread_mutex_lock(&g_telem_lock);
    bool landed = g_telem.have_landed_state &&
                  g_telem.landed_state == MAV_LANDED_STATE_ON_GROUND;
    pthread_mutex_unlock(&g_telem_lock);
    return landed;
}

/* sdk_check_startup_ready()
 *   Everything start() requires before it will report success:
 *   heartbeat, target sysid/component discovered, ATTITUDE,
 *   LOCAL_POSITION_NED, and GLOBAL_POSITION_INT all received at least
 *   once. Thread liveness is checked separately (sdk_threads_all_alive()
 *   in sdk_threads.h) since it isn't part of the telemetry cache. */
static inline bool sdk_check_startup_ready(const void *arg)
{
    (void)arg;
    if (!g_target_known) return false;
    pthread_mutex_lock(&g_telem_lock);
    bool ready = g_telem.have_heartbeat &&
                 g_telem.have_attitude &&
                 g_telem.local_pos.valid &&
                 g_telem.have_global_pos;
    pthread_mutex_unlock(&g_telem_lock);
    return ready;
}

#endif /* SDK_COMMANDS_H */
