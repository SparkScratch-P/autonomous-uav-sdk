/*
 * sdk_mavlink_recv.h - Decode incoming MAVLink messages into g_telem.
 *
 * This is the only place that writes to the telemetry cache. Every
 * write happens under g_telem_lock.
 */
#ifndef SDK_MAVLINK_RECV_H
#define SDK_MAVLINK_RECV_H

#include <string.h>
#include "sdk_state.h"
#include "sdk_log.h"

static inline gps_fix_t sdk_map_gps_fix(uint8_t mav_fix_type)
{
    switch (mav_fix_type) {
        case GPS_FIX_TYPE_2D_FIX:      return GPS_FIX_2D;
        case GPS_FIX_TYPE_3D_FIX:      return GPS_FIX_3D;
        case GPS_FIX_TYPE_DGPS:        return GPS_FIX_DGPS;
        case GPS_FIX_TYPE_RTK_FLOAT:
        case GPS_FIX_TYPE_RTK_FIXED:   return GPS_FIX_RTK;
        default:                       return GPS_FIX_NONE;
    }
}

/* sdk_handle_message()
 *   Called once per decoded MAVLink message by the telemetry thread.
 *   Also latches g_target_sysid/compid from the first HEARTBEAT seen
 *   from an autopilot component. */
static inline void sdk_handle_message(const mavlink_message_t *msg)
{
    switch (msg->msgid) {

    case MAVLINK_MSG_ID_HEARTBEAT: {
        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(msg, &hb);
        if (hb.autopilot == MAV_AUTOPILOT_INVALID) break; /* ignore GCS/companion heartbeats */

        if (!g_target_known) {
            g_target_sysid  = msg->sysid;
            g_target_compid = msg->compid;
            g_target_known  = true;
            SDK_LOG("telemetry: autopilot found (sysid=%d compid=%d)", msg->sysid, msg->compid);
        }
        if (msg->sysid != g_target_sysid || msg->compid != g_target_compid) break;

        pthread_mutex_lock(&g_telem_lock);
        g_telem.armed             = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
        g_telem.custom_mode       = hb.custom_mode;
        g_telem.base_mode         = hb.base_mode;
        g_telem.system_status_raw = hb.system_status;
        clock_gettime(CLOCK_MONOTONIC, &g_telem.last_heartbeat);
        g_telem.have_heartbeat    = true;
        pthread_mutex_unlock(&g_telem_lock);
        break;
    }

    case MAVLINK_MSG_ID_ATTITUDE: {
        mavlink_attitude_t a;
        mavlink_msg_attitude_decode(msg, &a);
        pthread_mutex_lock(&g_telem_lock);
        g_telem.roll_rad     = a.roll;
        g_telem.pitch_rad    = a.pitch;
        g_telem.yaw_rad      = a.yaw;
        g_telem.have_attitude = true;
        pthread_mutex_unlock(&g_telem_lock);
        break;
    }

    case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
        mavlink_local_position_ned_t p;
        mavlink_msg_local_position_ned_decode(msg, &p);
        pthread_mutex_lock(&g_telem_lock);
        g_telem.local_pos.x = p.x;
        g_telem.local_pos.y = p.y;
        g_telem.local_pos.z = p.z;
        g_telem.local_pos.vx = p.vx;
        g_telem.local_pos.vy = p.vy;
        g_telem.local_pos.vz = p.vz;
        g_telem.local_pos.valid = true;
        pthread_mutex_unlock(&g_telem_lock);
        break;
    }

    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        mavlink_global_position_int_t g;
        mavlink_msg_global_position_int_decode(msg, &g);
        pthread_mutex_lock(&g_telem_lock);
        g_telem.lat            = g.lat / 1e7;
        g_telem.lon            = g.lon / 1e7;
        g_telem.alt_msl_m      = g.alt / 1000.0f;
        g_telem.alt_rel_m      = g.relative_alt / 1000.0f;
        g_telem.heading_cdeg   = (g.hdg == UINT16_MAX) ? -1 : (int16_t)g.hdg;
        g_telem.have_global_pos = true;
        pthread_mutex_unlock(&g_telem_lock);
        break;
    }

    case MAVLINK_MSG_ID_HOME_POSITION: {
        mavlink_home_position_t h;
        mavlink_msg_home_position_decode(msg, &h);
        pthread_mutex_lock(&g_telem_lock);
        g_telem.home.lat   = h.latitude / 1e7;
        g_telem.home.lon   = h.longitude / 1e7;
        g_telem.home.alt_m = h.altitude / 1000.0f;
        g_telem.home.valid = true;
        pthread_mutex_unlock(&g_telem_lock);
        break;
    }

    case MAVLINK_MSG_ID_GPS_RAW_INT: {
        mavlink_gps_raw_int_t g;
        mavlink_msg_gps_raw_int_decode(msg, &g);
        pthread_mutex_lock(&g_telem_lock);
        g_telem.fix_type           = sdk_map_gps_fix(g.fix_type);
        g_telem.satellites_visible = g.satellites_visible;
        pthread_mutex_unlock(&g_telem_lock);
        break;
    }

    case MAVLINK_MSG_ID_SYS_STATUS: {
        mavlink_sys_status_t s;
        mavlink_msg_sys_status_decode(msg, &s);
        pthread_mutex_lock(&g_telem_lock);
        bool ekf_present = (s.onboard_control_sensors_present & MAV_SYS_STATUS_AHRS) != 0;
        bool ekf_healthy = (s.onboard_control_sensors_health  & MAV_SYS_STATUS_AHRS) != 0;
        g_telem.ekf_ok     = !ekf_present || ekf_healthy;
        g_telem.sensors_ok = (s.onboard_control_sensors_health & s.onboard_control_sensors_enabled)
                              == s.onboard_control_sensors_enabled;
        g_telem.battery.voltage_v = (s.voltage_battery == UINT16_MAX)
                                     ? -1.0f : s.voltage_battery / 1000.0f;
        g_telem.battery.current_a = (s.current_battery < 0)
                                     ? -1.0f : s.current_battery / 100.0f;
        g_telem.battery.remaining_pct = s.battery_remaining;
        g_telem.battery.valid = true;
        pthread_mutex_unlock(&g_telem_lock);
        break;
    }

    case MAVLINK_MSG_ID_BATTERY_STATUS: {
        mavlink_battery_status_t b;
        mavlink_msg_battery_status_decode(msg, &b);
        pthread_mutex_lock(&g_telem_lock);
        g_telem.battery.remaining_pct = b.battery_remaining;
        g_telem.battery.valid = true;
        pthread_mutex_unlock(&g_telem_lock);
        break;
    }

    case MAVLINK_MSG_ID_EXTENDED_SYS_STATE: {
        mavlink_extended_sys_state_t e;
        mavlink_msg_extended_sys_state_decode(msg, &e);
        pthread_mutex_lock(&g_telem_lock);
        g_telem.landed_state      = e.landed_state;
        g_telem.have_landed_state = true;
        pthread_mutex_unlock(&g_telem_lock);
        break;
    }

    case MAVLINK_MSG_ID_RC_CHANNELS: {
        mavlink_rc_channels_t rc;
        mavlink_msg_rc_channels_decode(msg, &rc);
        uint16_t raw[18] = {
            rc.chan1_raw, rc.chan2_raw, rc.chan3_raw, rc.chan4_raw,
            rc.chan5_raw, rc.chan6_raw, rc.chan7_raw, rc.chan8_raw,
            rc.chan9_raw, rc.chan10_raw, rc.chan11_raw, rc.chan12_raw,
            rc.chan13_raw, rc.chan14_raw, rc.chan15_raw, rc.chan16_raw,
            rc.chan17_raw, rc.chan18_raw,
        };
        pthread_mutex_lock(&g_telem_lock);
        memcpy(g_telem.rc.raw_us, raw, sizeof(raw));
        g_telem.rc.count = (uint8_t)rc.chancount;
        g_telem.rc.valid = true;
        pthread_mutex_unlock(&g_telem_lock);
        break;
    }

    case MAVLINK_MSG_ID_STATUSTEXT: {
        mavlink_statustext_t st;
        mavlink_msg_statustext_decode(msg, &st);
        size_t n = sizeof(st.text) < sizeof(g_telem.status_text) - 1
                       ? sizeof(st.text) : sizeof(g_telem.status_text) - 1;
        pthread_mutex_lock(&g_telem_lock);
        memcpy(g_telem.status_text, st.text, n);
        g_telem.status_text[n] = '\0';
        pthread_mutex_unlock(&g_telem_lock);
        SDK_LOG("PX4: %s", g_telem.status_text);
        break;
    }

    case MAVLINK_MSG_ID_COMMAND_ACK: {
        mavlink_command_ack_t ack;
        mavlink_msg_command_ack_decode(msg, &ack);
        pthread_mutex_lock(&g_telem_lock);
        g_telem.ack_command = ack.command;
        g_telem.ack_result  = ack.result;
        g_telem.ack_pending = false;
        pthread_cond_broadcast(&g_telem.ack_cond);
        pthread_mutex_unlock(&g_telem_lock);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_CURRENT: {
        mavlink_mission_current_t mc;
        mavlink_msg_mission_current_decode(msg, &mc);
        pthread_mutex_lock(&g_telem_lock);
        g_telem.mission_current_seq = mc.seq;
        pthread_mutex_unlock(&g_telem_lock);
        break;
    }

    case MAVLINK_MSG_ID_ODOMETRY: {
        mavlink_odometry_t o;
        mavlink_msg_odometry_decode(msg, &o);
        pthread_mutex_lock(&g_telem_lock);
        g_telem.vx = o.vx;
        g_telem.vy = o.vy;
        g_telem.vz = o.vz;
        g_telem.have_odometry = true;
        pthread_mutex_unlock(&g_telem_lock);
        break;
    }

    default:
        break; /* not a message we cache */
    }
}

#endif /* SDK_MAVLINK_RECV_H */
