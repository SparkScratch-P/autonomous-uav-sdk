/*
 * mav.c - Example autonomous mission.
 *
 * Every MAVLink detail, telemetry cache, thread, Offboard stream,
 * acknowledgement wait, and safety check lives inside func.h /
 * include/sdk_*.h. This file only calls the public API.
 */
#include "func.h"

int main(void)
{
    if (!start("/dev/ttyUSB0", 921600)) return 1;

    set_geofence(47.397742, 8.545594, 300.0f);
    set_max_altitude(30.0f);

    arm();
    takeoff(200);                              /* climb to 2 m */
    gps_go(47.398000, 8.545800, 10.0f, 3.0f);  /* fly to waypoint at 10 m, 3 m/s */
    yaw_to(90);                                /* face east */
    hover(3000);                               /* hold 3 s */
    move_forward(5);                           /* nudge forward 5 m */
    land();

    stop();
    return 0;
}
