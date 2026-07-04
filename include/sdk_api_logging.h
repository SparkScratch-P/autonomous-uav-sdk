/*
 * sdk_api_logging.h - Toggles the SDK_LOG() output used throughout
 * the library.
 */
#ifndef SDK_API_LOGGING_H
#define SDK_API_LOGGING_H

#include "sdk_state.h"

static inline void enable_logging(void)  { g_logging_enabled = true; }
static inline void disable_logging(void) { g_logging_enabled = false; }

#endif /* SDK_API_LOGGING_H */
