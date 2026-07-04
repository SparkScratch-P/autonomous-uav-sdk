/*
 * sdk_types.h - Public enums and structs returned/accepted by the API.
 *
 * PX4 (not ArduPilot) only. Flight modes below map onto PX4's
 * main_mode / sub_mode custom_mode encoding, documented in PX4
 * Firmware's px4_custom_mode.h.
 */
#ifndef SDK_TYPES_H
#define SDK_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------ */
/* Flight modes (PX4 native, not ArduPilot)                            */
/* ------------------------------------------------------------------ */
typedef enum {
    MODE_MANUAL,
    MODE_STABILIZED,
    MODE_ALTITUDE,
    MODE_POSITION,
    MODE_HOLD,
    MODE_MISSION,
    MODE_OFFBOARD,
    MODE_RETURN,
    MODE_LAND,
    MODE_UNKNOWN,
} flight_mode_t;

/* PX4 main/sub mode numbers (from PX4 Firmware px4_custom_mode.h) */
#define PX4_MAIN_MANUAL          1
#define PX4_MAIN_ALTCTL          2
#define PX4_MAIN_POSCTL          3
#define PX4_MAIN_AUTO            4
#define PX4_MAIN_OFFBOARD        6
#define PX4_MAIN_STABILIZED      7

#define PX4_SUB_AUTO_LOITER      3
#define PX4_SUB_AUTO_MISSION     4
#define PX4_SUB_AUTO_RTL         5
#define PX4_SUB_AUTO_LAND        6

/* Encode (main_mode, sub_mode) into the uint32_t custom_mode field
 * exactly the way PX4 Firmware's union px4_custom_mode packs it:
 * byte0-1 reserved, byte2 = main_mode, byte3 = sub_mode. */
#define PX4_CUSTOM_MODE(main, sub) \
    ( ((uint32_t)(main) << 16) | ((uint32_t)(sub) << 24) )

/* ------------------------------------------------------------------ */
/* Telemetry value objects                                             */
/* ------------------------------------------------------------------ */

typedef struct {
    double lat;         /* degrees */
    double lon;          /* degrees */
    float  alt_msl_m;     /* meters, mean sea level */
    float  alt_rel_m;      /* meters, above home */
    bool   valid;
} gps_location_t;

typedef struct {
    float x, y, z;        /* meters, NED, relative to local origin */
    float vx, vy, vz;      /* m/s, NED */
    bool  valid;
} local_position_t;

typedef struct {
    float  voltage_v;
    float  current_a;
    int8_t remaining_pct;   /* -1 if unknown */
    bool   valid;
} battery_status_t;

typedef struct {
    double lat, lon;
    float  alt_m;
    bool   valid;
} home_position_t;

typedef struct {
    bool armed;
    bool ekf_ok;
    flight_mode_t mode;
    bool heartbeat_ok;
    uint8_t system_status;   /* raw MAV_STATE */
} system_status_t;

typedef enum {
    GPS_FIX_NONE = 0,
    GPS_FIX_2D,
    GPS_FIX_3D,
    GPS_FIX_DGPS,
    GPS_FIX_RTK,
} gps_fix_t;

typedef struct {
    uint16_t raw_us[18];
    uint8_t  count;
    bool     valid;
} rc_channels_t;

#endif /* SDK_TYPES_H */
