/*
 * sdk_time_util.h - sleep_ms() and kick_watchdog().
 *
 * Every blocking wait loop in the SDK calls kick_watchdog() on each
 * iteration so the watchdog thread knows the mission logic is alive.
 */
#ifndef SDK_TIME_UTIL_H
#define SDK_TIME_UTIL_H

#include <time.h>
#include "sdk_state.h"

static inline void sleep_ms(int ms)
{
    if (ms <= 0) return;
    struct timespec ts;
    ts.tv_sec  = ms / 1000;
    ts.tv_nsec = (long)(ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}

static inline void kick_watchdog(void)
{
    pthread_mutex_lock(&g_kick_lock);
    clock_gettime(CLOCK_MONOTONIC, &g_last_kick);
    pthread_mutex_unlock(&g_kick_lock);
}

#endif /* SDK_TIME_UTIL_H */
