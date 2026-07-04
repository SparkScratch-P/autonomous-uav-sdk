# PX4 Autonomous Flight SDK (C, header-only)

Pure-C, header-only autonomous flight SDK for Pixhawk 4 / PX4, driven from a
Jetson (or any Linux companion computer) over MAVLink/UART. Offboard-only —
there is no MANUAL_CONTROL, no RC-stick emulation, and no time-based movement
anywhere in the library. Every closed-loop function confirms its result from
telemetry, never from elapsed time.

## Install

```
git clone git clone https://github.com/mavlink/c_library_v2.git
git clone https://github.com/SparkScratch-P/autonomous-uav-sdk.git
```

Extract the `c_library_v2` folder within the SDK Folder and start coding in `main.c`. 


## Build

```
gcc -Wall -Wextra -Wpedantic -Werror -O2 -o mav mav.c \
    -Iinclude -Ic_library_v2 -lpthread -lm
```

Verified to compile with **zero warnings** under `-Wall -Wextra -Wpedantic
-Werror` (and `-Wshadow`), for every function in the public API, not just the
example `mav.c`.

`c_library_v2/` is the unmodified upstream MAVLink v2 C library
(github.com/mavlink/c_library_v2). Its generated headers are not themselves
pedantic-clean (31st-bit enum values, `&packed_struct->field` patterns) — see
the comment in `func.h` around the include for how that's isolated with a
`#pragma GCC diagnostic push/pop` so it doesn't affect the strictness of the
SDK's own code.

## Layout

```
mav.c                       your mission — the only file you write
func.h                       master header, includes everything below in order
include/
  sdk_config.h                tunable constants
  sdk_types.h                  public enums/structs (flight_mode_t, gps_location_t, ...)
  sdk_state.h                   all global state + the telemetry cache
  sdk_log.h                      SDK_LOG()
  sdk_error.h                     rich sdk_error_t codes, get_last_error()
  sdk_fsm.h                         internal sdk_state_t lifecycle FSM (guards every command)
  sdk_recorder.h                     CSV flight recorder (every command, timestamped)
  sdk_time_util.h                     sleep_ms(), kick_watchdog()
  sdk_serial.h                         serial port open/close/send (partial-write & EAGAIN safe)
  sdk_mavlink_recv.h                    decode -> telemetry cache
  sdk_mavlink_send.h                     encode outbound MAVLink messages
  sdk_target.h                            shared "what should Offboard send right now"
  sdk_commands.h                           ACK waits (serialized) + closed-loop condition waits
  sdk_threads.h                             the 5 background threads (validated startup, alive flags)
  sdk_api_core.h                             start/stop/arm/mode
  sdk_api_flight.h                            takeoff/land/rtl/hold/emergency_brake
  sdk_api_altitude.h                           altitude control
  sdk_api_speed.h                               cruise-speed tunable + readback
  sdk_api_local.h                                local/body-frame navigation
  sdk_api_motion.h                                move_forward/back/left/right
  sdk_api_velocity.h                               raw velocity control
  sdk_api_yaw.h                                     heading control
  sdk_api_navigation.h                               GPS waypoint navigation
  sdk_api_telemetry.h                                 readback getters
  sdk_api_mission.h                                    pause/resume/abort/wait
  sdk_api_safety.h                                      speed/altitude/geofence limits
  sdk_api_rc.h                                           RC autonomy-switch override
  sdk_api_config.h                                        mission defaults
  sdk_api_logging.h                                        log on/off
tests/
  test_sdk.c                  logic-only unit tests, no PX4/serial hardware needed
```

`mav.c` never includes MAVLink, never touches a thread, never parses a
packet. It calls `start()`, `arm()`, `takeoff()`, `gps_go()`, `land()`,
`stop()` — that's the whole contract.

## Internal lifecycle state machine, error codes, and the flight recorder

Every public function still returns `bool` and takes the same arguments as
before — this pass added robustness underneath the existing API, not new
surface area. Three things now sit underneath every call:

- **An internal `sdk_state_t` lifecycle** (`SDK_DISCONNECTED`, `SDK_CONNECTING`,
  `SDK_READY`, `SDK_ARMED`, `SDK_OFFBOARD`, `SDK_MISSION`, `SDK_HOLD`,
  `SDK_LANDING`, `SDK_RTL`, `SDK_EMERGENCY`, `SDK_ABORTED`, `SDK_STOPPED`),
  defined and enforced in `sdk_fsm.h`. Every command-sending function
  (`arm()`, `takeoff()`, `gps_go()`, `land()`, `velocity()`, ...) checks it
  first via the `SDK_REQUIRE_STATE(mask)` guard in `sdk_commands.h`: an
  illegal call (e.g. `gps_go()` before `start()`, `arm()` while already
  armed and flying) fails immediately with `SDK_BAD_STATE` and sends no
  MAVLink traffic, instead of hanging in a wait loop or racing PX4 into an
  undefined situation. Every legal sequence the API already documented
  (`start → arm → takeoff → gps_go → ... → land → stop`) walks a legal path
  through this machine by construction, so no mission code needs to change.
  `get_sdk_state()` / `get_sdk_state_name()` expose it read-only for
  logging/introspection; mission code was never required to touch it.
- **`get_last_error()`** returns an `sdk_error_t` (`SDK_OK`, `SDK_TIMEOUT`,
  `SDK_ACK_REJECTED`, `SDK_OFFBOARD_LOST`, `SDK_RC_OVERRIDE`,
  `SDK_ERR_ABORTED`, `SDK_CONNECTION_LOST`, `SDK_BAD_STATE`,
  `SDK_NO_TELEMETRY`) set right before any `false` return, anywhere in the
  SDK. `get_last_error_string()` gives the human-readable form.
- **The flight recorder** (`sdk_recorder.h`) writes a timestamped CSV to
  `flight_<unix-time>.csv` automatically from `start()` to `stop()` — every
  command, every arrival/failure, every state transition, every RC
  override, Offboard-loss, and `BAD_STATE` rejection. Columns:
  `ms_since_start,event,detail`.

## Production-readiness hardening

This pass focused entirely on robustness, determinism, and thread-safety —
no architecture, public API, navigation algorithm, or mission-facing
behavior changed.

- **Serialized COMMAND_ACK transactions.** `g_command_lock`
  (`sdk_commands.h`) is held for the full send-through-wait-for-ACK duration
  of every `sdk_send_command_and_wait()` call, so at most one such
  transaction is ever outstanding, regardless of how many threads call into
  the SDK concurrently (mission thread, RC monitor thread, watchdog thread).
  Since MAVLink's `COMMAND_ACK` carries no per-transaction id, serialization
  — not clever matching — is what actually makes "this ACK belongs to that
  command" true by construction instead of by coincidence. (The one residual
  gap, documented in `sdk_commands.h`, is a MAVLink protocol limitation, not
  an SDK bug: a late ACK from a timed-out command can in principle still be
  mistaken for the ACK of an immediately-following retry of the *same*
  command id — no amount of SDK-side bookkeeping can close that without a
  custom out-of-band nonce PX4 doesn't send back.)
- **`start()` now gates on real readiness, not just a heartbeat.** It
  doesn't return `true` until HEARTBEAT, ATTITUDE, LOCAL_POSITION_NED, and
  GLOBAL_POSITION_INT have all been received at least once, the target
  system/component has been discovered, *and* every background thread has
  confirmed it's actually running (see below) — all within
  `SDK_STARTUP_READY_TIMEOUT_MS`. If any condition isn't met, `start()`
  fails and leaves the SDK exactly as if it had never been called.
- **Thread creation is validated, with rollback.** `sdk_threads_start()`
  (internal, not public API) checks every `pthread_create()` return value.
  If any thread fails to start, every thread already started is signaled to
  exit and joined before `start()` reports failure — the SDK is never left
  half-initialized with some threads running and others missing. Each
  thread also sets a dedicated `_alive` flag right before entering its main
  loop (and clears it right before returning), so "thread running" means
  the thread is actually executing, not just that `pthread_create()`
  returned `0`.
- **`stop()` leaves the SDK genuinely reusable.** Beyond joining every
  thread, clearing the Offboard target, disabling the Offboard-active flag,
  and closing the serial port (all present before), `stop()` now also
  resets the discovered target sysid/component and every telemetry
  "have I received this yet" flag. Without that reset, a `stop()` followed
  by a fresh `start()` (e.g. reconnecting after a link drop, or to a
  different vehicle) could pass the readiness gate immediately on stale
  data left over from the previous session before any new telemetry
  actually arrived — that's now closed.
- **Serial writes handle partial writes and repeated failures instead of
  ignoring them.** `sdk_serial_write_all()` loops until every byte is
  written, retrying transient `EAGAIN`/`EWOULDBLOCK`/`EINTR` (expected on
  the non-blocking fd at high setpoint-stream rates) with a bounded
  backoff, and treats a real transport error (`EIO`, `ENODEV`, ...) or an
  exhausted retry budget as failure rather than a silently-dropped byte.
  `SDK_SERIAL_MAX_CONSECUTIVE_FAILURES` back-to-back failures — a genuinely
  down link, not a momentarily full buffer — sets `SDK_CONNECTION_LOST` and
  moves the lifecycle to `SDK_EMERGENCY` so the rest of the SDK (and the
  flight recorder) can see it instead of flying blind on a dead transport.

`tests/test_sdk.c` covers all of the above without needing PX4/SITL/hardware
— see Testing below.

## Testing

`tests/test_sdk.c` is a logic-only unit test suite: mode encode/decode
round-trips, the closed-loop condition checks (`sdk_check_alt_rel`,
`sdk_check_local_arrival`, `sdk_check_heading`, `sdk_check_startup_ready`),
the great-circle distance math, the speed/altitude/geofence clamps, the
lifecycle state machine and its `SDK_REQUIRE_STATE` guards (illegal calls
rejected immediately with `SDK_BAD_STATE`), the error-code plumbing, a real
`start()` failure path against a nonexistent serial device (with thread
rollback and lifecycle reset verified), `sdk_serial_write_all()` under
forced partial-write/`EAGAIN` conditions against a real pipe, and a
concurrency test proving `g_command_lock` actually serializes simultaneous
`COMMAND_ACK` transactions (three concurrent timeout-bound calls take
~3× one call's timeout, not ~1×). None of it needs PX4, SITL, or hardware:

```
cd tests && gcc -Wall -Wextra -Wpedantic -Werror -O2 -o test_sdk test_sdk.c \
    -I../include -I../c_library_v2 -lpthread -lm && ./test_sdk
```

That's necessary but not sufficient — it validates the SDK's own logic, not
its behavior against a real autopilot. Once you have PX4 SITL + Gazebo
running, point `start()` at its MAVLink bridge and walk through the
scenarios listed at the top of `test_sdk.c` (arm, Offboard entry, takeoff,
hover, GPS navigation, velocity commands, emergency brake, RC override,
RTL, landing, companion disconnect, Offboard loss) for real. A mission
queue (`enqueue()`/`cancel()`/`pause()`/`resume()`/`clear()` for
asynchronous, composable missions) is a natural next layer on top of this,
but isn't implemented here — the SDK is currently synchronous by design,
and a queue is easy to add once there's real flight-test data showing what
shape it needs to be.

## Threads

Heartbeat, telemetry reader, Offboard setpoint streaming, watchdog, and RC
monitor — five threads, started by `start()` and joined by `stop()`. The
watchdog thread also now watches HEARTBEAT freshness directly (not just the
mission loop's kick_watchdog() calls) and freezes position the same way on
either kind of staleness. (The design brief also listed a separate "mission
timing" thread; `mission_time()` and `flight_time()` are implemented as
direct monotonic-clock reads instead, since a sixth thread that only
updated a value nothing else consumed would be dead weight under the
project's own "zero dead code" requirement. "Safe shutdown" is `stop()`'s
job, not a thread.)

## Notable design decisions worth knowing about

- **`emergency_stop()` is an alias for `emergency_brake()`.** The SDK never
  cuts motors in flight in response to a software condition — forcing a
  disarm mid-air is itself a crash. Freezing position in place is the safe
  maximal automatic response; landing (`land()`) or disarming (`disarm()`)
  remain separate, explicit calls so a human or mission layer decides.
- **`SDK_EMERGENCY` vs `SDK_ABORTED`** are deliberately distinct lifecycle
  states: `SDK_EMERGENCY` is where `emergency_brake()`, a watchdog trip
  (mission loop unresponsive or PX4 heartbeat stale), or a persistently
  failing serial link land — "something is actively wrong, frozen in
  place." `SDK_ABORTED` is where an RC autonomy-switch cutoff or PX4 
  leaving OFFBOARD on its own land — "the pilot or PX4 itself ended
  autonomy." Both are included in `hold_position()`'s and `rtl()`'s legal
  state set, since recovering out of either by explicitly holding or
  returning is exactly the action you want to still be available.
- **`land()` vs `land_at_rate(rate)`** replace the pseudocode's overloaded
  `land()`/`land(rate)` — C has no overloading. `land()` hands the descent to
  PX4's own AUTO.LAND; `land_at_rate()` drives a closed-loop Offboard
  velocity descent at a caller-chosen rate. Both confirm touchdown via
  `EXTENDED_SYS_STATE` and confirm disarm before returning `true`.
- **`gps_go(lat, lon, alt_m, speed_mps)`** takes `SDK_ALT_KEEP_CURRENT` /
  `SDK_SPEED_KEEP_CURRENT` sentinels in place of the pseudocode's multiple
  overloaded signatures, for the same reason.
- **Safety limits are companion-side.** `set_max_speed()`,
  `set_max_altitude()`, and `set_geofence()` clamp values before they're ever
  sent to PX4 (see `sdk_api_safety.h`). Configure PX4's own onboard
  parameters too for a real vehicle — this is a second layer, not a
  replacement.
- **`restart()`** always returns `false` with a log explaining why: there is
  no portable way to recover a serial device path from an already-open file
  descriptor. Call `stop()` then `start(port, baud)` explicitly instead.
- **`get_air_speed()`** returns the same value as `get_speed()`/
  `get_ground_speed()` — a multirotor has no airspeed sensor to report
  differently; the function exists for API symmetry.

## PX4 only

Flight-mode encoding uses PX4's native `main_mode`/`sub_mode` custom_mode
packing (documented in PX4 Firmware's `px4_custom_mode.h`), not ArduPilot's
mode-number table. No ArduPilot compatibility is implemented.
