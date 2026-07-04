/*
 * func.h - Autonomous flight SDK for Pixhawk/PX4, PX4 Offboard only.
 *
 * mav.c should never need to know anything about MAVLink, threads,
 * serial ports, telemetry parsing, or Offboard streaming. It just
 * calls the functions declared across the sdk_api_*.h files that this
 * header pulls in, for example:
 *
 *     start("/dev/ttyUSB0", 921600);
 *     arm();
 *     takeoff(150);
 *     gps_go(47.397, 8.545, 20.0f, 3.0f);
 *     yaw_to(90);
 *     hover(3000);
 *     land();
 *     stop();
 *
 * ARCHITECTURE
 *   sdk_config.h          tunable constants
 *   sdk_types.h            public enums/structs
 *   sdk_state.h             all global state + the telemetry cache
 *   sdk_log.h                 SDK_LOG()
 *   sdk_error.h                rich sdk_error_t codes, get_last_error()
 *   sdk_fsm.h                   internal sdk_state_t lifecycle FSM (guards every command)
 *   sdk_recorder.h                CSV flight recorder (every command, timestamped)
 *   sdk_time_util.h                sleep_ms(), kick_watchdog()
 *   sdk_serial.h                 serial port open/close/send (partial-write & EAGAIN safe)
 *   sdk_mavlink_recv.h             decode -> telemetry cache
 *   sdk_mavlink_send.h              encode outbound messages
 *   sdk_target.h                     shared "what should Offboard send"
 *   sdk_commands.h                    ACK waits (serialized via g_command_lock) + closed-loop waits
 *   sdk_threads.h                      the 5 background threads (validated startup, alive flags)
 *   sdk_api_core.h                      start/stop/arm/mode
 *   sdk_api_flight.h                     takeoff/land/rtl/hold/brake
 *   sdk_api_altitude.h                    altitude control
 *   sdk_api_speed.h                        speed tunable + readback
 *   sdk_api_local.h                         local/body-frame navigation
 *   sdk_api_motion.h                         move_forward/back/left/right
 *   sdk_api_velocity.h                        raw velocity control
 *   sdk_api_yaw.h                              heading control
 *   sdk_api_navigation.h                        GPS waypoint navigation
 *   sdk_api_telemetry.h                          readback getters
 *   sdk_api_mission.h                             pause/resume/abort/wait
 *   sdk_api_safety.h                               speed/alt/geofence limits
 *   sdk_api_rc.h                                    autonomy RC override
 *   sdk_api_config.h                                 mission defaults
 *   sdk_api_logging.h                                 log on/off
 *
 * Every autonomous movement in this SDK uses PX4 Offboard
 * (SET_POSITION_TARGET_LOCAL_NED / SET_POSITION_TARGET_GLOBAL_INT).
 * There is no MANUAL_CONTROL, no stick emulation, and no time-based
 * movement anywhere in this library - every wait_until_*()/closed-loop
 * function confirms its result from telemetry.
 *
 * PX4 only. Not ArduPilot compatible.
 *
 * Build:
 *   gcc -Wall -Wextra -Wpedantic -Werror -O2 -o mav mav.c \
 *       -Iinclude -Ic_library_v2/common -Ic_library_v2 -lpthread -lm
 */
#ifndef FUNC_H
#define FUNC_H

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <time.h>
#include <math.h>

/* The upstream MAVLink v2 C library (c_library_v2) is not itself
 * -Wpedantic / -Waddress-of-packed-member clean (it uses >31-bit enum
 * values and takes addresses of fields inside __attribute__((packed))
 * structs, both by design). Those warnings originate in generated
 * code we do not own and cannot edit, so they are suppressed only for
 * the duration of this one include - everything in sdk_*.h is still
 * compiled at full -Wall -Wextra -Wpedantic -Werror strictness. */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include "c_library_v2/common/mavlink.h"
#pragma GCC diagnostic pop

#include "sdk_config.h"
#include "sdk_types.h"
#include "sdk_state.h"
#include "sdk_log.h"
#include "sdk_error.h"
#include "sdk_fsm.h"
#include "sdk_recorder.h"
#include "sdk_time_util.h"
#include "sdk_serial.h"
#include "sdk_mavlink_recv.h"
#include "sdk_mavlink_send.h"
#include "sdk_target.h"
#include "sdk_commands.h"
#include "sdk_threads.h"

#include "sdk_api_core.h"
#include "sdk_api_flight.h"
#include "sdk_api_safety.h"
#include "sdk_api_config.h"
#include "sdk_api_altitude.h"
#include "sdk_api_speed.h"
#include "sdk_api_local.h"
#include "sdk_api_motion.h"
#include "sdk_api_velocity.h"
#include "sdk_api_yaw.h"
#include "sdk_api_navigation.h"
#include "sdk_api_telemetry.h"
#include "sdk_api_mission.h"
#include "sdk_api_rc.h"
#include "sdk_api_logging.h"

#endif /* FUNC_H */
