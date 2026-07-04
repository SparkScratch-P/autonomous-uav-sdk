/*
 * sdk_config.h - All tunable constants for the autonomous flight SDK.
 *
 * Change values here, not inside the logic files.
 */
#ifndef SDK_CONFIG_H
#define SDK_CONFIG_H

/* --- MAVLink identity --------------------------------------------- */
#define SDK_SYSTEM_ID          255
#define SDK_COMPONENT_ID       MAV_COMP_ID_ONBOARD_COMPUTER

/* --- Serial ---------------------------------------------------------*/
#define SDK_DEFAULT_BAUD        115200
#define SDK_SERIAL_WRITE_RETRY_LIMIT        50   /* EAGAIN retries per send before giving up */
#define SDK_SERIAL_WRITE_RETRY_DELAY_US       500 /* backoff between retries */
#define SDK_SERIAL_MAX_CONSECUTIVE_FAILURES  20   /* persistent-failure threshold */

/* --- Thread periods (milliseconds) --------------------------------- */
#define SDK_HEARTBEAT_PERIOD_MS     1000   /* our own heartbeat to PX4 */
#define SDK_OFFBOARD_STREAM_HZ        20   /* PX4 requires >= 2 Hz, we use 20 */
#define SDK_OFFBOARD_STREAM_MS      (1000 / SDK_OFFBOARD_STREAM_HZ)
#define SDK_WATCHDOG_TIMEOUT_MS      3000  /* mission loop must "kick" within this */
#define SDK_WATCHDOG_POLL_MS          200
#define SDK_RC_MONITOR_POLL_MS        100
#define SDK_MISSION_TIMER_POLL_MS     200

/* --- Offboard pre-stream requirement -------------------------------- */
#define SDK_OFFBOARD_PRESTREAM_MS     500  /* PX4 needs setpoints flowing before it will accept OFFBOARD */

/* --- Command / mode timeouts (milliseconds) -------------------------*/
#define SDK_COMMAND_ACK_TIMEOUT_MS   2000
#define SDK_ARM_TIMEOUT_MS           5000
#define SDK_MODE_TIMEOUT_MS          5000
#define SDK_HEARTBEAT_WAIT_TIMEOUT_MS 10000
#define SDK_STARTUP_READY_TIMEOUT_MS  15000  /* heartbeat + target + attitude + local/global pos + all threads alive */
#define SDK_LAND_TIMEOUT_MS          60000

/* --- Closed loop tolerances ------------------------------------------*/
#define SDK_ALT_TOLERANCE_M            0.30f
#define SDK_POSITION_TOLERANCE_M       0.50f
#define SDK_HEADING_TOLERANCE_DEG      5.0f
#define SDK_SPEED_TOLERANCE_MPS        0.20f
#define SDK_LANDED_VZ_TOLERANCE_MPS    0.10f

/* --- Default flight tunables (overridable via sdk_api_config.h) -----*/
#define SDK_DEFAULT_TAKEOFF_ALT_M      1.5f
#define SDK_DEFAULT_CRUISE_SPEED_MPS   2.0f
#define SDK_DEFAULT_YAW_SPEED_DEGPS    30.0f
#define SDK_DEFAULT_DESCENT_RATE_MPS   0.5f

/* --- Safety defaults --------------------------------------------------*/
#define SDK_DEFAULT_MAX_SPEED_MPS      8.0f
#define SDK_DEFAULT_MAX_ALT_M          120.0f
#define SDK_RC_AUTONOMY_CHANNEL        6      /* CH6: autonomy enable switch */
#define SDK_RC_AUTONOMY_ON_US          1500   /* PWM above this = autonomy ON */

/* --- Earth model (for lat/lon distance math) ------------------------*/
#define SDK_EARTH_RADIUS_M       6371000.0

/* --- gps_go() sentinels: "leave this axis alone" --------------------*/
#define SDK_ALT_KEEP_CURRENT     (-9999.0f)
#define SDK_SPEED_KEEP_CURRENT   (-1.0f)
#define SDK_GPS_GO_DEFAULT_RADIUS_M   1.0f

#endif /* SDK_CONFIG_H */
