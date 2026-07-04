/*
 * sdk_api_motion.h - The four cardinal body-frame moves. Thin,
 * readable wrappers over goto_body() - exactly what a 30-line
 * mission script wants to call.
 */
#ifndef SDK_API_MOTION_H
#define SDK_API_MOTION_H

#include "sdk_api_local.h"

static inline bool move_forward(float meters)  { return goto_body(meters, 0.0f, 0.0f); }
static inline bool move_backward(float meters) { return goto_body(-meters, 0.0f, 0.0f); }
static inline bool move_left(float meters)     { return goto_body(0.0f, -meters, 0.0f); }
static inline bool move_right(float meters)    { return goto_body(0.0f, meters, 0.0f); }

#endif /* SDK_API_MOTION_H */
