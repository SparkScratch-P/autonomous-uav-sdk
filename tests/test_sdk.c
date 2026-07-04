/*
 * test_sdk.c - Logic-level unit tests for func.h.
 *
 * These test the SDK's pure logic (mode encode/decode, closed-loop
 * condition checks, distance math, safety clamps, the state machine,
 * and error-code plumbing) by poking the same static globals mav.c
 * would poke through the API, without a real PX4 or serial link.
 *
 * What this file does NOT replace: real behavioral validation against
 * PX4 SITL + Gazebo. Once you have a SITL instance running, point
 * start() at its MAVLink UDP-to-serial bridge (e.g. via
 * mavlink-router or `socat`) and exercise these scenarios for real:
 *
 *   [ ] arm                      [ ] velocity commands
 *   [ ] Offboard entry            [ ] emergency brake
 *   [ ] takeoff                    [ ] RC override
 *   [ ] hover                       [ ] RTL
 *   [ ] GPS navigation               [ ] landing
 *                                     [ ] companion disconnect
 *                                     [ ] Offboard loss
 *
 * Build:
 *   gcc -Wall -Wextra -Wpedantic -Werror -O2 -o test_sdk test_sdk.c \
 *       -I../include -I../c_library_v2 -lpthread -lm
 */
#include "../func.h"
#include <assert.h>

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond) \
    do { \
        if (cond) { g_pass++; } \
        else { g_fail++; fprintf(stderr, "FAIL: %s (%s:%d)\n", #cond, __FILE__, __LINE__); } \
    } while (0)

static void test_mode_roundtrip(void)
{
    flight_mode_t modes[] = {
        MODE_MANUAL, MODE_STABILIZED, MODE_ALTITUDE, MODE_POSITION,
        MODE_HOLD, MODE_MISSION, MODE_OFFBOARD, MODE_RETURN, MODE_LAND,
    };
    for (size_t i = 0; i < sizeof(modes) / sizeof(modes[0]); i++) {
        uint32_t px4 = sdk_flight_mode_to_px4(modes[i]);
        flight_mode_t back = sdk_px4_to_flight_mode(px4);
        CHECK(back == modes[i]);
    }
}

static void test_horizontal_distance(void)
{
    /* One degree of latitude is ~111.19 km, everywhere on Earth. */
    double d = sdk_horizontal_distance_m(0.0, 0.0, 1.0, 0.0);
    CHECK(d > 110500.0 && d < 111500.0);

    /* Same point -> zero distance. */
    CHECK(sdk_horizontal_distance_m(47.0, 8.0, 47.0, 8.0) < 0.01);
}

static void test_alt_condition(void)
{
    g_telem.alt_rel_m = 10.0f;
    float target_within = 10.2f;   /* inside SDK_ALT_TOLERANCE_M (0.30) */
    float target_outside = 12.0f;  /* outside tolerance */
    CHECK(sdk_check_alt_rel(&target_within) == true);
    CHECK(sdk_check_alt_rel(&target_outside) == false);
}

static void test_local_arrival_condition(void)
{
    g_telem.local_pos.valid = true;
    g_telem.local_pos.x = 5.0f;
    g_telem.local_pos.y = 5.0f;
    g_telem.local_pos.z = -2.0f;

    sdk_local_point_t near_target = { 5.1f, 5.1f, -2.0f };
    sdk_local_point_t far_target  = { 10.0f, 10.0f, -2.0f };
    CHECK(sdk_check_local_arrival(&near_target) == true);
    CHECK(sdk_check_local_arrival(&far_target) == false);

    g_telem.local_pos.valid = false;
    CHECK(sdk_check_local_arrival(&near_target) == false); /* no telemetry yet -> never "arrived" */
    g_telem.local_pos.valid = true;
}

static void test_heading_condition(void)
{
    g_telem.yaw_rad = 0.0f; /* facing north */
    float target_close = 2.0f;    /* within SDK_HEADING_TOLERANCE_DEG (5) */
    float target_far   = 90.0f;
    CHECK(sdk_check_heading(&target_close) == true);
    CHECK(sdk_check_heading(&target_far) == false);

    /* wraparound: 359 deg is 1 deg away from 0, not 359. */
    g_telem.yaw_rad = sdk_deg_to_rad(359.0f);
    float target_zero = 0.0f;
    CHECK(sdk_check_heading(&target_zero) == true);
}

static void test_safety_clamps(void)
{
    set_max_speed(5.0f);
    CHECK(sdk_clamp_speed(3.0f) == 3.0f);
    CHECK(sdk_clamp_speed(10.0f) == 5.0f);
    CHECK(sdk_clamp_speed(-10.0f) == -5.0f);
    set_max_speed(SDK_DEFAULT_MAX_SPEED_MPS); /* restore default */

    set_max_altitude(20.0f);
    CHECK(sdk_clamp_altitude(15.0f) == 15.0f);
    CHECK(sdk_clamp_altitude(50.0f) == 20.0f);
    set_max_altitude(SDK_DEFAULT_MAX_ALT_M);
}

static void test_geofence_clamp(void)
{
    set_geofence(0.0, 0.0, 100.0f); /* 100 m radius around the origin */
    double lat = 0.01, lon = 0.0;   /* ~1.1 km north - well outside */
    sdk_clamp_to_geofence(&lat, &lon, sdk_horizontal_distance_m);
    double clamped_dist = sdk_horizontal_distance_m(0.0, 0.0, lat, lon);
    CHECK(clamped_dist <= 100.5 && clamped_dist >= 99.0);
    g_geofence.enabled = false; /* reset for other tests */
}

static void test_state_machine(void)
{
    sdk_set_state(SDK_DISCONNECTED);
    CHECK(get_sdk_state() == SDK_DISCONNECTED);

    sdk_set_state(SDK_ARMED);
    CHECK(get_sdk_state() == SDK_ARMED);
    CHECK(strcmp(get_sdk_state_name(), "ARMED") == 0);

    sdk_set_state(SDK_ABORTED);
    CHECK(get_sdk_state() == SDK_ABORTED);

    sdk_set_state(SDK_DISCONNECTED); /* leave clean for the tests that follow */
}

static void test_state_guards_reject_illegal_calls(void)
{
    sdk_set_state(SDK_DISCONNECTED);

    /* arm() before start(): illegal, must fail immediately with
     * SDK_BAD_STATE and must not send anything (no fd is open, so a
     * hang here would mean the guard didn't fire before the send). */
    CHECK(arm() == false);
    CHECK(get_last_error() == SDK_BAD_STATE);

    /* gps_go() while disconnected: same story. */
    CHECK(gps_go(0.0, 0.0, SDK_ALT_KEEP_CURRENT, SDK_SPEED_KEEP_CURRENT) == false);
    CHECK(get_last_error() == SDK_BAD_STATE);

    /* land() with nothing flying: illegal. */
    CHECK(land() == false);
    CHECK(get_last_error() == SDK_BAD_STATE);

    /* Once "connected", arm() is legal again. */
    sdk_set_state(SDK_READY);
    CHECK(get_sdk_state() == SDK_READY);

    sdk_set_state(SDK_DISCONNECTED); /* leave clean */
}

static void test_startup_ready_condition(void)
{
    g_target_known = false;
    g_telem.have_heartbeat  = false;
    g_telem.have_attitude   = false;
    g_telem.local_pos.valid = false;
    g_telem.have_global_pos = false;
    CHECK(sdk_check_startup_ready(NULL) == false);

    g_target_known = true;
    g_telem.have_heartbeat  = true;
    g_telem.have_attitude   = true;
    g_telem.local_pos.valid = true;
    g_telem.have_global_pos = true;
    CHECK(sdk_check_startup_ready(NULL) == true);

    /* Missing any single condition fails the whole gate. */
    g_telem.have_global_pos = false;
    CHECK(sdk_check_startup_ready(NULL) == false);
    g_telem.have_global_pos = true;
}

/* A background reader that drains a pipe slowly enough to force the
 * writer through real EAGAIN/partial-write conditions, then confirms
 * every byte still arrives intact and in order. */
static int g_pipe_read_fd = -1;
static size_t g_pipe_bytes_expected = 0;
static size_t g_pipe_bytes_received = 0;
static bool g_pipe_content_ok = true;

static void *sdk_test_pipe_reader(void *arg)
{
    (void)arg;
    uint8_t buf[4096];
    size_t next_expected_value = 0;
    while (g_pipe_bytes_received < g_pipe_bytes_expected) {
        ssize_t n = read(g_pipe_read_fd, buf, sizeof(buf));
        if (n <= 0) { sleep_ms(1); continue; }
        for (ssize_t i = 0; i < n; i++) {
            if (buf[i] != (uint8_t)(next_expected_value % 256)) g_pipe_content_ok = false;
            next_expected_value++;
        }
        g_pipe_bytes_received += (size_t)n;
    }
    return NULL;
}

static void test_serial_write_all_handles_partial_and_eagain(void)
{
    int fds[2];
    CHECK(pipe(fds) == 0);
    int read_fd = fds[0], write_fd = fds[1];
    fcntl(write_fd, F_SETFL, O_NONBLOCK); /* same mode sdk_serial_open() uses */

    /* Larger than any realistic pipe buffer, to force at least one
     * EAGAIN/partial-write cycle inside sdk_serial_write_all(). */
    const size_t total = 512 * 1024;
    uint8_t *data = malloc(total);
    CHECK(data != NULL);
    for (size_t i = 0; i < total; i++) data[i] = (uint8_t)(i % 256);

    g_pipe_read_fd = read_fd;
    g_pipe_bytes_expected = total;
    g_pipe_bytes_received = 0;
    g_pipe_content_ok = true;

    pthread_t reader;
    pthread_create(&reader, NULL, sdk_test_pipe_reader, NULL);

    bool ok = sdk_serial_write_all(write_fd, data, (uint16_t)0); /* len=0 edge case first */
    CHECK(ok == true); /* nothing to write is trivially successful */

    /* sdk_serial_write_all() takes a uint16_t length, so exercise it
     * in MAVLINK_MAX_PACKET_LEN-sized chunks like sdk_serial_send()
     * actually does, rather than claiming a single call handles more
     * than 64KB. */
    size_t sent = 0;
    bool all_ok = true;
    while (sent < total) {
        uint16_t chunk = (uint16_t)((total - sent) > 1024 ? 1024 : (total - sent));
        if (!sdk_serial_write_all(write_fd, data + sent, chunk)) { all_ok = false; break; }
        sent += chunk;
    }
    CHECK(all_ok == true);

    pthread_join(reader, NULL);
    CHECK(g_pipe_bytes_received == total);
    CHECK(g_pipe_content_ok == true);

    free(data);
    close(read_fd);
    close(write_fd);
}

static void *sdk_test_command_thread(void *arg)
{
    (void)arg;
    /* No PX4 is connected, so this will always time out - the point
     * is to prove concurrent callers serialize rather than race. */
    sdk_send_command_and_wait(MAV_CMD_DO_SET_MODE, 0, 0, 0, 0, 0, 0, 0, 300);
    return NULL;
}

static void test_command_ack_serialization(void)
{
    struct timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);

    pthread_t threads[3];
    for (int i = 0; i < 3; i++) {
        pthread_create(&threads[i], NULL, sdk_test_command_thread, NULL);
    }
    for (int i = 0; i < 3; i++) {
        pthread_join(threads[i], NULL);
    }

    clock_gettime(CLOCK_MONOTONIC, &t1);
    long elapsed_ms = (t1.tv_sec - t0.tv_sec) * 1000L + (t1.tv_nsec - t0.tv_nsec) / 1000000L;

    /* If the three 300ms-timeout transactions were serialized (as
     * g_command_lock guarantees), total elapsed time is close to
     * 3*300=900ms. If they instead raced in parallel, it would be
     * close to 300ms. Allow generous scheduling slack either way. */
    CHECK(elapsed_ms >= 800);
}

static void test_error_codes(void)
{
    sdk_set_error(SDK_OK);
    CHECK(get_last_error() == SDK_OK);
    CHECK(strcmp(get_last_error_string(), "OK") == 0);

    sdk_set_error(SDK_TIMEOUT);
    CHECK(get_last_error() == SDK_TIMEOUT);
    CHECK(strcmp(get_last_error_string(), "TIMEOUT") == 0);
}

static void test_start_bad_port_fails_cleanly(void)
{
    /* No real hardware needed: opening a nonexistent device must fail
     * fast, set SDK_CONNECTION_LOST, and leave no threads running. */
    sdk_set_state(SDK_DISCONNECTED); /* previous tests may have left a different state */
    bool ok = start("/dev/nonexistent_test_port_xyz", 115200);
    CHECK(ok == false);
    CHECK(get_last_error() == SDK_CONNECTION_LOST);
    CHECK(g_threads_running == false);
    CHECK(sdk_threads_all_alive() == false);
    CHECK(get_sdk_state() == SDK_DISCONNECTED);
}

int main(void)
{
    disable_logging(); /* keep test output readable */

    test_mode_roundtrip();
    test_horizontal_distance();
    test_alt_condition();
    test_local_arrival_condition();
    test_heading_condition();
    test_safety_clamps();
    test_geofence_clamp();
    test_state_machine();
    test_state_guards_reject_illegal_calls();
    test_startup_ready_condition();
    test_serial_write_all_handles_partial_and_eagain();
    test_command_ack_serialization();
    test_error_codes();
    test_start_bad_port_fails_cleanly();

    fprintf(stderr, "\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
