/*
 * sdk_recorder.h - Persists a timestamped trace of the flight to a
 * CSV file, independent of the stderr debug log (SDK_LOG). This is
 * what you pull up after a field test to see exactly what the SDK
 * did and when.
 *
 * Opened automatically by start(), closed by stop(). Columns:
 *   ms_since_start, event, detail
 */
#ifndef SDK_RECORDER_H
#define SDK_RECORDER_H

#include <stdio.h>
#include <stdarg.h>
#include <pthread.h>
#include <time.h>
#include "sdk_log.h"

static FILE *g_recorder_fp = NULL;
static pthread_mutex_t g_recorder_lock = PTHREAD_MUTEX_INITIALIZER;

/* sdk_recorder_open()
 *   path may be NULL to disable recording (SDK_RECORD() becomes a
 *   no-op). Safe to call even if a previous recorder is still open -
 *   closes it first. */
static inline void sdk_recorder_open(const char *path)
{
    pthread_mutex_lock(&g_recorder_lock);
    if (g_recorder_fp) {
        fclose(g_recorder_fp);
        g_recorder_fp = NULL;
    }
    if (path) {
        g_recorder_fp = fopen(path, "w");
        if (g_recorder_fp) {
            fprintf(g_recorder_fp, "ms_since_start,event,detail\n");
            fflush(g_recorder_fp);
        }
    }
    pthread_mutex_unlock(&g_recorder_lock);
    if (path && !g_recorder_fp) {
        SDK_LOG("recorder: failed to open %s, flight recording disabled", path);
    }
}

static inline void sdk_recorder_close(void)
{
    pthread_mutex_lock(&g_recorder_lock);
    if (g_recorder_fp) {
        fclose(g_recorder_fp);
        g_recorder_fp = NULL;
    }
    pthread_mutex_unlock(&g_recorder_lock);
}

__attribute__((format(printf, 2, 3)))
static inline void sdk_recorder_write(const char *event, const char *fmt, ...)
{
    pthread_mutex_lock(&g_recorder_lock);
    if (g_recorder_fp) {
        fprintf(g_recorder_fp, "%ld,%s,", sdk_now_ms_since_start(), event);
        va_list ap;
        va_start(ap, fmt);
        vfprintf(g_recorder_fp, fmt, ap);
        va_end(ap);
        fprintf(g_recorder_fp, "\n");
        fflush(g_recorder_fp);
    }
    pthread_mutex_unlock(&g_recorder_lock);
}

#define SDK_RECORD(event, ...) sdk_recorder_write((event), __VA_ARGS__)

#endif /* SDK_RECORDER_H */
