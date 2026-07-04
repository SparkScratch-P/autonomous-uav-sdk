/*
 * sdk_error.h - Rich error reporting.
 *
 * Every public function still returns bool (matches the original API
 * contract mav.c is written against), but on failure it also records
 * *why* here via sdk_set_error(). Call get_last_error() right after a
 * false return to find out what actually happened - critical during
 * field testing, where "it returned false" alone tells you nothing.
 *
 * Not thread-local: in a single mission thread calling into the SDK
 * (the normal usage pattern) that's irrelevant. If you call SDK
 * functions from multiple mission threads concurrently, treat
 * get_last_error() as racy and read it immediately after the call
 * that failed, on the same thread, before any other SDK call runs.
 */
#ifndef SDK_ERROR_H
#define SDK_ERROR_H

#include <pthread.h>

typedef enum {
    SDK_OK = 0,
    SDK_TIMEOUT,          /* an ACK or a telemetry-confirmed state never arrived in time */
    SDK_ACK_REJECTED,      /* PX4 answered but refused the command (MAV_RESULT != ACCEPTED) */
    SDK_OFFBOARD_LOST,      /* PX4 left OFFBOARD mode on its own mid-command */
    SDK_RC_OVERRIDE,         /* the pilot's autonomy-enable switch cut off the command */
    SDK_ERR_ABORTED,          /* emergency_brake()/watchdog/stop_requested() cut the command short */
    SDK_CONNECTION_LOST,       /* HEARTBEAT went stale */
    SDK_BAD_STATE,              /* called out of order, e.g. arm() before start() */
    SDK_NO_TELEMETRY,            /* needed a telemetry field that hasn't arrived yet */
} sdk_error_t;

static pthread_mutex_t g_error_lock = PTHREAD_MUTEX_INITIALIZER;
static sdk_error_t g_last_error = SDK_OK;

static inline void sdk_set_error(sdk_error_t err)
{
    pthread_mutex_lock(&g_error_lock);
    g_last_error = err;
    pthread_mutex_unlock(&g_error_lock);
}

/* get_last_error()
 *   The sdk_error_t behind the most recent false/failure return from
 *   any SDK call on this thread. SDK_OK after a call that succeeded. */
static inline sdk_error_t get_last_error(void)
{
    pthread_mutex_lock(&g_error_lock);
    sdk_error_t v = g_last_error;
    pthread_mutex_unlock(&g_error_lock);
    return v;
}

static inline const char *sdk_error_string(sdk_error_t err)
{
    switch (err) {
        case SDK_OK:              return "OK";
        case SDK_TIMEOUT:         return "TIMEOUT";
        case SDK_ACK_REJECTED:    return "ACK_REJECTED";
        case SDK_OFFBOARD_LOST:   return "OFFBOARD_LOST";
        case SDK_RC_OVERRIDE:     return "RC_OVERRIDE";
        case SDK_ERR_ABORTED:     return "ABORTED";
        case SDK_CONNECTION_LOST: return "CONNECTION_LOST";
        case SDK_BAD_STATE:       return "BAD_STATE";
        case SDK_NO_TELEMETRY:    return "NO_TELEMETRY";
        default:                  return "UNKNOWN";
    }
}

/* get_last_error_string()
 *   Convenience: sdk_error_string(get_last_error()). */
static inline const char *get_last_error_string(void)
{
    return sdk_error_string(get_last_error());
}

#endif /* SDK_ERROR_H */
