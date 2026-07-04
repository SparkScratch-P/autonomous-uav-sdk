/*
 * sdk_log.h - Minimal logging. Timestamps are relative to start().
 */
#ifndef SDK_LOG_H
#define SDK_LOG_H

#include <stdio.h>
#include <time.h>
#include "sdk_state.h"

static inline long sdk_now_ms_since_start(void)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    long sec_diff = (long)(now.tv_sec - g_t0.tv_sec);
    long nsec_diff = now.tv_nsec - g_t0.tv_nsec;
    return sec_diff * 1000L + nsec_diff / 1000000L;
}

#define SDK_LOG(...) \
    do { \
        if (g_logging_enabled) { \
            fprintf(stderr, "[%6ld ms] ", sdk_now_ms_since_start()); \
            fprintf(stderr, __VA_ARGS__); \
            fprintf(stderr, "\n"); \
        } \
    } while (0)

#endif /* SDK_LOG_H */
