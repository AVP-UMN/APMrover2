# GCS_Mavlink.pde

Link to the code:[GCS_Mavlink.pde](https://github.com/BeaglePilot/ardupilot/blob/master/APMrover2/GCS_Mavlink.pde)

---

**MavLink** is a communication protocol for MAV (Micro Aerial Vehicles) that has nowadays been extended to all kind of drones (both aerial and terrestrial).

A **Ground Control station (GCS)** is a land- or sea-based control center that provides the facilities for human control of unmanned vehicles in the air or in space.When talking about a rover it can be as simple as a remote control device.

---

In this file the MavLink protocol is implemented and adapted to a rover.Remote control methods and sensors (rate control, attitude stabilization, yaw, altitud...) are enabled.

```cpp

// default sensors are present and healthy: gyro, accelerometer, rate_control, attitude_stabilization, yaw_position, altitude control, x/y position control, motor_control
#define MAVLINK_SENSOR_PRESENT_DEFAULT (MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL | MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION | MAV_SYS_STATUS_SENSOR_YAW_POSITION | MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL | MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS | MAV_SYS_STATUS_AHRS)

// use this to prevent recursion during sensor init
static bool in_mavlink_delay;

// true if we are out of time in our event timeslice
static bool	gcs_out_of_time;

// check if a message will fit in the payload space available
#define CHECK_PAYLOAD_SIZE(id) if (payload_space < MAVLINK_MSG_ID_## id ##_LEN) return false
...
```
Defines some variables and stablish the default configuration for sensors.

```cpp
/*
 *  !!NOTE!!
 *
 *  the use of NOINLINE separate functions for each message type avoids
 *  a compiler bug in gcc that would cause it to use far more stack
 *  space than is needed. Without the NOINLINE we use the sum of the
 *  stack needed for each message type. Please be careful to follow the
 *  pattern below when adding any new messages
 */

 static NOINLINE void send_heartbeat(mavlink_channel_t chan)
{
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint8_t system_status = MAV_STATE_ACTIVE;
    uint32_t custom_mode = control_mode;

    if (failsafe.triggered != 0) {
        system_status = MAV_STATE_CRITICAL;
    }
...
```
Stablishes the system status to ACTIVE and the control mode. Checks if the failsafe event is set (it means triggered() returns something different from 0). In case the failsafe event is set the status is changed to CRITICAL.

```cpp
...
    // work out the base_mode. This value is not very useful
    // for APM, but we calculate it as best we can so a generic
    // MAVLink enabled ground station can work out something about
    // what the MAV is up to. The actual bit values are highly
    // ambiguous for most of the APM flight modes. In practice, you
    // only get useful information from the custom_mode, which maps to
    // the APM flight mode and has a well defined meaning in the
    // ArduPlane documentation
    switch (control_mode) {
    case MANUAL:
    case LEARNING:
    case STEERING:
        base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        break;
    case AUTO:
    case RTL:
    case GUIDED:
        base_mode = MAV_MODE_FLAG_GUIDED_ENABLED;
// note that MAV_MODE_FLAG_AUTO_ENABLED does not match what
        // APM does in any mode, as that is defined as "system finds its own goal
        // positions", which APM does not currently do
        break;
    case INITIALISING:
        system_status = MAV_STATE_CALIBRATING;
        break;
    case HOLD:
        system_status = 0;
        break;
    }
...
```
Depending on the `control_mode` some variables are defined.The `MAV_MODE_GLAG` are defined as part of PX4 firmware, you can find the definitions [here](https://github.com/PX4/Firmware/blob/master/src/modules/commander/commander.cpp#L137).

- `MAV_MODE_FLAG_GUIDED_ENABLED`:guided mode enabled, system flies MISSIONs / mission items


- `MAV_MODE_FLAG_AUTO_ENABLED`:autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation.

At [ardupilot/libraries/GCS_MAVLink/include/mavlink/v1.0](https://github.com/diydrones/ardupilot/tree/master/libraries/GCS_MAVLink/include/mavlink/v1.0) you can find all these identifiers.
