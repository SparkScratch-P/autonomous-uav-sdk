/*
 * sdk_state.h - All process-wide state lives here, behind mutexes.
 *
 * Nothing outside this file touches these variables directly except
 * through the accessor functions in sdk_mavlink_recv.h (writers) and
 * sdk_api_telemetry.h (readers).
 */
#ifndef SDK_STATE_H
#define SDK_STATE_H

#include <pthread.h>
#include <signal.h>
#include <time.h>
#include "sdk_types.h"

/* --- Connection ------------------------------------------------------*/
static int g_fd = -1;
static uint8_t g_target_sysid  = 0;
static uint8_t g_target_compid = 0;
static volatile bool g_target_known = false;
static pthread_mutex_t g_serial_lock = PTHREAD_MUTEX_INITIALIZER;

/* --- Lifecycle ---------------------------------------------------------*/
static volatile sig_atomic_t g_request_stop = 0;
static volatile bool g_threads_running = false;
static volatile bool g_offboard_active = false;
static volatile bool g_logging_enabled = true;

/* --- Watchdog "kick" timestamp ------------------------------------------*/
static pthread_mutex_t g_kick_lock = PTHREAD_MUTEX_INITIALIZER;
static struct timespec g_last_kick;
static struct timespec g_t0; /* process start, for relative log timestamps */

/* --- Threads -----------------------------------------------------------*/
static pthread_t g_heartbeat_thread;
static pthread_t g_telemetry_thread;
static pthread_t g_offboard_thread;
static pthread_t g_watchdog_thread;
static pthread_t g_rc_monitor_thread;

/* Set true by each thread right before it enters its main loop, and
 * false right before it returns. start() waits on all five before
 * reporting success; sdk_threads_start() uses them to know which
 * threads actually came up if pthread_create() partially fails. */
static volatile bool g_heartbeat_thread_alive  = false;
static volatile bool g_telemetry_thread_alive  = false;
static volatile bool g_offboard_thread_alive   = false;
static volatile bool g_watchdog_thread_alive   = false;
static volatile bool g_rc_monitor_thread_alive = false;

/* --- Tunables settable at runtime via sdk_api_config.h ------------------*/
static pthread_mutex_t g_tunables_lock = PTHREAD_MUTEX_INITIALIZER;
static float g_takeoff_alt_m    = SDK_DEFAULT_TAKEOFF_ALT_M;
static float g_cruise_speed_mps = SDK_DEFAULT_CRUISE_SPEED_MPS;
static float g_yaw_speed_degps  = SDK_DEFAULT_YAW_SPEED_DEGPS;
static float g_max_speed_mps    = SDK_DEFAULT_MAX_SPEED_MPS;
static float g_max_alt_m        = SDK_DEFAULT_MAX_ALT_M;

/* --- Autonomy / RC override state ---------------------------------------*/
static volatile bool g_autonomy_enabled = true; /* software-level enable */

/* ------------------------------------------------------------------ */
/* Offboard target - what the streaming thread sends right now         */
/* ------------------------------------------------------------------ */
typedef enum {
    TARGET_NONE = 0,
    TARGET_LOCAL_POSITION,
    TARGET_LOCAL_VELOCITY,
    TARGET_GLOBAL_POSITION,
    TARGET_YAW_RATE,
} target_kind_t;

typedef struct {
    target_kind_t kind;
    float x, y, z;          /* local NED position (m) or velocity (m/s) */
    double lat, lon;        /* global position target */
    float alt_m;            /* global position target altitude (rel to home) */
    float yaw_rad;          /* NAN => hold current yaw */
    bool  has_yaw;
} offboard_target_t;

static pthread_mutex_t g_target_lock = PTHREAD_MUTEX_INITIALIZER;
static offboard_target_t g_target = { .kind = TARGET_NONE };

/* ------------------------------------------------------------------ */
/* Telemetry cache                                                     */
/* ------------------------------------------------------------------ */
typedef struct {
    /* HEARTBEAT */
    bool     armed;
    uint32_t custom_mode;
    uint8_t  base_mode;
    uint8_t  system_status_raw;
    struct timespec last_heartbeat;
    bool     have_heartbeat;

    /* ATTITUDE */
    float roll_rad, pitch_rad, yaw_rad;
    bool  have_attitude;

    /* LOCAL_POSITION_NED */
    local_position_t local_pos;

    /* GLOBAL_POSITION_INT */
    double lat, lon;
    float  alt_msl_m, alt_rel_m;
    int16_t heading_cdeg; /* centi-degrees, 0..35999 */
    bool   have_global_pos;

    /* HOME_POSITION */
    home_position_t home;

    /* GPS_RAW_INT */
    gps_fix_t fix_type;
    uint8_t   satellites_visible;

    /* SYS_STATUS */
    bool ekf_ok;
    bool sensors_ok;

    /* BATTERY_STATUS */
    battery_status_t battery;

    /* EXTENDED_SYS_STATE */
    uint8_t landed_state; /* MAV_LANDED_STATE_* */
    bool    have_landed_state;

    /* RC_CHANNELS */
    rc_channels_t rc;

    /* STATUSTEXT (most recent only) */
    char status_text[64];

    /* COMMAND_ACK (most recent) */
    pthread_cond_t  ack_cond;
    uint16_t        ack_command;
    uint8_t         ack_result;
    bool            ack_pending;

    /* MISSION_CURRENT */
    uint16_t mission_current_seq;

    /* ODOMETRY */
    float vx, vy, vz; /* m/s, NED, from ODOMETRY if present */
    bool  have_odometry;
} telemetry_cache_t;

static pthread_mutex_t g_telem_lock = PTHREAD_MUTEX_INITIALIZER;
static telemetry_cache_t g_telem;

#endif /* SDK_STATE_H */
