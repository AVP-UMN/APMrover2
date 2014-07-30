# APMrover2.pde

Link to the code: [APMrover2.pde](https://github.com/BeaglePilot/ardupilot/blob/master/APMrover2/APMrover2.pde).

---
The **firmware** is a block of machine instructions for specific purposes, engraved in memory, typically read / write (ROM, EEPROM, flash, etc..), Which provides the lowest level of logic that controls the electronic circuits of a device of any kind. It is tightly integrated with the electronic device being software that has direct interaction with the hardware: the manager with remote commands to execute properly.
In summary, a **firmware is software that manages the hardware physically**.

The **.pde extension** belongs to Processing Development Environment (PDE) (Source Code File) by *Processing*.
The PDE consists of a simple text editor for writing code, It is an open source programming language and environment for people who want to program images, animation, and interactions.You can read more [here](http://www.processing.org/).

---

In this file is the firmeware of `APMrover2`. All   variables, struct and functions needed are defined and implemented here:



```cpp
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define THISFIRMWARE "ArduRover v2.46beta2"
...
```
Here a firmware is defined with "ArduRover v2.46beta2" name.

```
...
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
...
/*
   This is the APMrover2 firmware. It was originally derived from
   ArduPlane by Jean-Louis Naudin (JLN), and then rewritten after the
   AP_HAL merge by Andrew Tridgell
...

```
This slice of code provides a description of the content and license, see full text in the `APMrover2.pde`file.
```cpp
...
// Radio setup:
// APM INPUT (Rec = receiver)
// Rec ch1: Steering
// Rec ch2: not used
// Rec ch3: Throttle
// Rec ch4: not used
// Rec ch5: not used
// Rec ch6: not used
// Rec ch7: Option channel to 2 position switch
// Rec ch8: Mode channel to 6 position switch
// APM OUTPUT
// Ch1: Wheel servo (direction)
// Ch2: not used
// Ch3: to the motor ESC
// Ch4: not used
...
``

The channels state / function is declared in the commet lines.

```cpp

////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include <stdarg.h>
#include <stdio.h>

// Libraries
#include <AP_Common.h> /*Common definitions and utility routines for the ArduPilot libraries.*/

#include <AP_Progmem.h>
#include <AP_HAL.h> /*AP_HAL consists of a set of headers (.h) that define the classes and methods that should be implemmented if ardupilot should run in a new device/architecture.*/

#include <AP_Menu.h> /* The Menu class implements a simple command line that accepts commands typed by
the user, and passes the arguments to those commands to a function defined as handing the command.*/

#include <AP_Param.h> //The AP variable store.

#include <AP_GPS.h>         // ArduPilot GPS library

#include <AP_ADC.h>         // ArduPilot Mega Analog to Digital Converter Library

#include <AP_ADC_AnalogSource.h> //Analog-digital conversion

#include <AP_Baro.h> //barometer methods

#include <AP_Compass.h>     // ArduPilot Mega Magnetometer Library

#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library

#include <AP_InertialSensor.h> // Inertial Sensor (uncalibated IMU) Library

#include <AP_AHRS.h>         // ArduPilot Mega DCM Library

#include <AP_NavEKF.h>  //Attitude and position estimation.

#include <AP_Mission.h>     // Mission command library

#include <AP_Terrain.h>  //Terrain grid

#include <PID.h>            // PID library

#include <RC_Channel.h>     // RC Channel Library

#include <AP_RangeFinder.h>	// Range finder library

#include <Filter.h>			// Filter library

#include <Butter.h>			// Filter library - butterworth filter

#include <AP_Buffer.h>      // FIFO buffer library

#include <ModeFilter.h>		// Mode Filter from Filter library

#include <AverageFilter.h>	// Mode Filter from Filter library

#include <AP_Relay.h>       // APM relay

#include <AP_ServoRelayEvents.h> /*handle DO_SET_SERVO, DO_REPEAT_SERVO, DO_SET_RELAY and
 * DO_REPEAT_RELAY commands */

#include <AP_Mount.h>		// Camera/Antenna mount

#include <AP_Camera.h>		// Camera triggering

#include <GCS_MAVLink.h>    // MAVLink GCS definitions

#include <AP_Airspeed.h>    // needed for AHRS build

#include <AP_Vehicle.h>     // needed for AHRS build

#include <DataFlash.h> //Test for DataFlash Log library

#include <AP_RCMapper.h>        // RC input mapping library

#include <SITL.h>  //SITL (software in the loop) simulator allows

#include <AP_Scheduler.h>       // main loop scheduler

#include <stdarg.h> /* macros to access the individual arguments of a list of unnamed arguments ( http://www.cplusplus.com/reference/cstdarg/ ) */

#include <AP_Navigation.h> //generic interface for navigation controllers.

#include <APM_Control.h> // methods for roll, pitch, yaw and steer control.

#include <AP_L1_Control.h> /*Explicit control over tack control angle ,frequency and damping */

#include <AP_BoardConfig.h> //board specific configuration

/*imports AP_HAL instances for the processes indicated by the name*/
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_VRBRAIN.h>
#include <AP_HAL_FLYMAPLE.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_Empty.h>

#include "compat.h" /* set of various pseudo-constants dedicated to increase compatibility between the TI-89 and TI-92 Plus and between AMS versions.*/

#include <AP_Notify.h>      // Notify library
#include <AP_BattMonitor.h> // Battery monitor library

// Configuration
#include "config.h"

// Local modules
#include "defines.h"
#include "Parameters.h"
#include "GCS.h"

#include <AP_Declination.h> // ArduPilot Mega Declination Helper Library
...
```
This code has been modified for adding elucidative notes beside the headers inclusion.

All the headers can be found in [ardupilot/libraries](https://github.com/BeaglePilot/ardupilot/tree/master/libraries) or in [Standard C++ Library reference](http://www.cplusplus.com/reference/).

```cpp
...
AP_HAL::BetterStream* cliSerial;

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
...

```
In this code we find two definitions:

- The const `AP_HAL::HAL& hal` is here defined, and can be later exported.


- `cliSerial` is a BetterStream type pointer.
You can find the code of `BetterStream` at [AP_HAL/utility/BetterStream.h](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_HAL/utility/BetterStream.h); this code provides some implementations for AVR microcontrollers.

```cpp
// this sets up the parameter table, and sets the default values. This
// must be the first AP_Param variable declared to ensure its
// constructor runs before the constructors of the other AP_Param
// variables
AP_Param param_loader(var_info, MISSION_START_BYTE);
...
```
You can see the implementation of this constructor in [AP_Param.h](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_Param/AP_Param.h#L108).

```cpp

////////////////////////////////////////////////////////////////////////////////
// the rate we run the main loop at
////////////////////////////////////////////////////////////////////////////////
static const AP_InertialSensor::Sample_rate ins_sample_rate = AP_InertialSensor::RATE_50HZ;
...
```
The [AP_InertialSensor](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_InertialSensor) is an abstraction for gyro and accelerometer measurements which are correctly aligned to the body axes and scaled to SI units.In the `AP_InertialSensor.h` code we find the `Sample_rate` defintion.
```
 // the rate that updates will be available to the application
    enum Sample_rate {
        RATE_50HZ,
        RATE_100HZ,
        RATE_200HZ,
        RATE_400HZ
    };
```
Continuing with the `APMrover2.pde` code we find definitions for parameters.
```

////////////////////////////////////////////////////////////////////////////////
// Parameters
////////////////////////////////////////////////////////////////////////////////
//
// Global parameters are all contained within the 'g' class.
//
static Parameters      g;

// main loop scheduler
static AP_Scheduler scheduler;

// mapping between input channels
static RCMapper rcmap;

// board specific config
static AP_BoardConfig BoardConfig;

// primary control channels
static RC_Channel *channel_steer;
static RC_Channel *channel_throttle;
static RC_Channel *channel_learn;
...
```
```cpp

////////////////////////////////////////////////////////////////////////////////
// prototypes
static void update_events(void);
void gcs_send_text_fmt(const prog_char_t *fmt, ...);
static void print_mode(AP_HAL::BetterStream *port, uint8_t mode);
...
```
After the parameters some functions for working with prototypes are defined.

```cpp

////////////////////////////////////////////////////////////////////////////////
// DataFlash
////////////////////////////////////////////////////////////////////////////////
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
static DataFlash_APM1 DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM2
static DataFlash_APM2 DataFlash;
#elif defined(HAL_BOARD_LOG_DIRECTORY)
static DataFlash_File DataFlash(HAL_BOARD_LOG_DIRECTORY);
#else
DataFlash_Empty DataFlash;
#endif

static bool in_log_download;
...
```
Next, the `Dataflash` options are configured depending on the board type.Note that `DataFlash` is a low pin-count serial interface for flash memory.

```cpp

////////////////////////////////////////////////////////////////////////////////
// Sensors
////////////////////////////////////////////////////////////////////////////////
//
// There are three basic options related to flight sensor selection.
//
// - Normal driving mode.  Real sensors are used.
// - HIL Attitude mode.  Most sensors are disabled, as the HIL
//   protocol supplies attitude information directly.
// - HIL Sensors mode.  Synthetic sensors are configured that
//   supply data from the simulation.
//

// GPS driver
static AP_GPS gps;

// flight modes convenience array
static AP_Int8		*modes = &g.mode1;
...
```
The sensors are initialized:

- `gps`is defined as [AP_GPS](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_GPS/AP_GPS.h#L60) class instance.


- `AP_Int8` is part of [APMrover2/Parameters.h](https://github.com/BeaglePilot/ardupilot/blob/master/APMrover2/Parameters.h).

```cpp
...

#if CONFIG_BARO == HAL_BARO_BMP085
static AP_Baro_BMP085 barometer;
#elif CONFIG_BARO == HAL_BARO_PX4
static AP_Baro_PX4 barometer;
#elif CONFIG_BARO == HAL_BARO_VRBRAIN
static AP_Baro_VRBRAIN barometer;
#elif CONFIG_BARO == HAL_BARO_HIL
static AP_Baro_HIL barometer;
#elif CONFIG_BARO == HAL_BARO_MS5611
static AP_Baro_MS5611 barometer(&AP_Baro_MS5611::i2c);
#elif CONFIG_BARO == HAL_BARO_MS5611_SPI
static AP_Baro_MS5611 barometer(&AP_Baro_MS5611::spi);
#else
 #error Unrecognized CONFIG_BARO setting
#endif
...
```
The barometer defined at [AP_Baro.h](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_Baro/AP_Baro.h#L93) is configured according to the barometer type.Notice that a barometer measures atmospheric pressure.

```cpp
...
#if CONFIG_COMPASS == HAL_COMPASS_PX4
static AP_Compass_PX4 compass;
#elif CONFIG_COMPASS == HAL_COMPASS_VRBRAIN
static AP_Compass_VRBRAIN compass;
#elif CONFIG_COMPASS == HAL_COMPASS_HMC5843
static AP_Compass_HMC5843 compass;
#elif CONFIG_COMPASS == HAL_COMPASS_HIL
static AP_Compass_HIL compass;
#else
 #error Unrecognized CONFIG_COMPASS setting
#endif
...
```
Defines the compass using  [AP_Compass.h](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_Compass/AP_Compass.h#L6) which catchs-all header that defines all supported compass classes.Also includes HIL methods.

**Note** about **HIL**: Simulation Hardware-in-the-loop (HIL) is a technique used for the development and testing of embedded real-time systems complex.

```cpp
...
#if CONFIG_INS_TYPE == HAL_INS_OILPAN || CONFIG_HAL_BOARD == HAL_BOARD_APM1
AP_ADC_ADS7844 apm1_adc;
#endif
...
```
Stablish `apm_adc` of [AP_ADC_ADS7844](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_ADC/AP_ADC_ADS7844.h) class for  reading values of the sensors.

```cpp
...
#if CONFIG_INS_TYPE == HAL_INS_MPU6000
AP_InertialSensor_MPU6000 ins;
#elif CONFIG_INS_TYPE == HAL_INS_PX4
AP_InertialSensor_PX4 ins;
#elif CONFIG_INS_TYPE == HAL_INS_VRBRAIN
AP_InertialSensor_VRBRAIN ins;
#elif CONFIG_INS_TYPE == HAL_INS_HIL
AP_InertialSensor_HIL ins;
#elif CONFIG_INS_TYPE == HAL_INS_OILPAN
AP_InertialSensor_Oilpan ins( &apm1_adc );
#elif CONFIG_INS_TYPE == HAL_INS_FLYMAPLE
AP_InertialSensor_Flymaple ins;
#elif CONFIG_INS_TYPE == HAL_INS_L3G4200D
AP_InertialSensor_L3G4200D ins;
#elif CONFIG_INS_TYPE == HAL_INS_MPU9250
AP_InertialSensor_MPU9250 ins;
#else
  #error Unrecognised CONFIG_INS_TYPE setting.
#endif // CONFIG_INS_TYPE
...
```
Defines the [InertialSensor](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_InertialSensor/AP_InertialSensor.h) values for gyro and accelerometer measurements.

```cpp
...

// Inertial Navigation EKF
#if AP_AHRS_NAVEKF_AVAILABLE
AP_AHRS_NavEKF ahrs(ins, barometer, gps);
#else
AP_AHRS_DCM ahrs(ins, barometer, gps);
#endif
...
```
Defines the [AP_AHRS](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_AHRS/AP_AHRS.h) values for AHRS (Attitude Heading Reference System) interface for ArduPilot.


```cpp
...

static AP_L1_Control L1_controller(ahrs);

// selected navigation controller
static AP_Navigation *nav_controller = &L1_controller;

// steering controller
static AP_SteerController steerController(ahrs);
...
```
Some instances for `AP_L1_controller`, `AP_Navigation` and `AP_steerController` respectively are defined.

- [AP_L1_Control](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_L1_Control/AP_L1_Control.h) for :
    + Explicit control over frequency and damping.
    + Explicit control over track capture angle.
    + Ability to use loiter radius smaller than L1 length.


- [AP_Navigation](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_Navigation/AP_Navigation.h) defines a generic interface for navigation controllers.


- [AP_steerController](https://github.com/diydrones/ardupilot/blob/master/libraries/APM_Control/AP_SteerController.h) returns a steering servo output.

```
...
////////////////////////////////////////////////////////////////////////////////
// Mission library
// forward declaration to avoid compiler errors
////////////////////////////////////////////////////////////////////////////////
static bool start_command(const AP_Mission::Mission_Command& cmd);
static bool verify_command(const AP_Mission::Mission_Command& cmd);
static void exit_mission();
AP_Mission mission(ahrs, &start_command, &verify_command, &exit_mission, MISSION_START_BYTE, MISSION_END_BYTE);

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
SITL sitl;
#endif
...
```

Defines [AP_Mission](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_Mission) methods. `AP_Mission` library performances are:

- responsible for managing a list of commands made up of "nav", "do" and "conditional" commands.


- reads and writes the mission commands to storage.


- provides easy acces to current, previous and upcoming waypoints.


- calls main program's command execution and verify functions.


- accounts for the `DO_JUMP` command.

```cpp


////////////////////////////////////////////////////////////////////////////////
// GCS selection
////////////////////////////////////////////////////////////////////////////////
//
static const uint8_t num_gcs = MAVLINK_COMM_NUM_BUFFERS;
static GCS_MAVLINK gcs[MAVLINK_COMM_NUM_BUFFERS];

// a pin for reading the receiver RSSI voltage. The scaling by 0.25
// is to take the 0 to 1024 range down to an 8 bit range for MAVLink
AP_HAL::AnalogSource *rssi_analog_source;
...
```

`MAVLinks`presents a communication protocol for MAV (Micro Aerial Vehicles). [GCS.h](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/GCS_MAVLink/GCS.h) stablish how message are sent to Ground Control Station (GCS) using MAVLink protocol.

```cpp
////////////////////////////////////////////////////////////////////////////////
// SONAR
static RangeFinder sonar;

// relay support
AP_Relay relay;

AP_ServoRelayEvents ServoRelayEvents(relay);

// Camera
#if CAMERA == ENABLED
static AP_Camera camera(&relay);
#endif

// The rover's current location
static struct 	Location current_loc;


// Camera/Antenna mount tracking and stabilisation stuff
// --------------------------------------
#if MOUNT == ENABLED
// current_loc uses the baro/gps soloution for altitude rather than gps only.
AP_Mount camera_mount(&current_loc, ahrs, 0);
#endif
...
```
Un this slice of code `AP_Relay`and `AP_ServoRelayEvents` appear:

- [AP_Relay.h](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_Relay/AP_Relay.h) provides a APM relay control class.


- [AP_ServoRElayEvents](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_ServoRelayEvents/AP_ServoRelayEvents.h) handles some commands for servo controllling.

The **camera is activated** calling an `AP_Camera`class constructor. [AP_Camera.h](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_Camera/AP_Camera.h)
is a photo or video camera manager, with EEPROM-backed storage of constants.

As the camera is enabled, also the **mount must be enabled**.[AP_Mount](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_Mount/AP_Mount.h) moves a 2 or 3 axis mount attached to vehicle.Its main use is for mount to track targets or stabilise camera plus other modes.

```cpp

////////////////////////////////////////////////////////////////////////////////
// Global variables
////////////////////////////////////////////////////////////////////////////////

// if USB is connected
static bool usb_connected;

/* Radio values
		Channel assignments
			1   Steering
			2   ---
			3   Throttle
			4   ---
			5   Aux5
			6   Aux6
			7   Aux7/learn
			8   Aux8/Mode
		Each Aux channel can be configured to have any of the available auxiliary functions assigned to it.
		See libraries/RC_Channel/RC_Channel_aux.h for more information
*/
...
```

This slice of code indicates the state/use of the channels  in case the USB is connected.

```cpp
////////////////////////////////////////////////////////////////////////////////
// Radio
////////////////////////////////////////////////////////////////////////////////
// This is the state of the flight control system
// There are multiple states defined such as MANUAL, FBW-A, AUTO
enum mode   control_mode        = INITIALISING;
// Used to maintain the state of the previous control switch position
// This is set to -1 when we need to re-read the switch
uint8_t 	oldSwitchPosition;
// These are values received from the GCS if the user is using GCS joystick
// control and are substituted for the values coming from the RC radio
static int16_t rc_override[8] = {0,0,0,0,0,0,0,0};
// A flag if GCS joystick control is in use
static bool rc_override_active = false;
...
```
 Here the radio- control parameters are initialized.

```cpp
////////////////////////////////////////////////////////////////////////////////
// Failsafe
////////////////////////////////////////////////////////////////////////////////
// A tracking variable for type of failsafe active
// Used for failsafe based on loss of RC signal or GCS signal. See
// FAILSAFE_EVENT_*
static struct {
    uint8_t bits;
    uint32_t rc_override_timer;
    uint32_t start_time;
    uint8_t triggered;
    uint32_t last_valid_rc_ms;
} failsafe;

// notification object for LEDs, buzzers etc (parameter set to false disables external leds)
static AP_Notify notify;

// A counter used to count down valid gps fixes to allow the gps estimate to settle
// before recording our home position (and executing a ground start if we booted with an air start)
static uint8_t 	ground_start_count	= 20;
...
```
A **fail-safe** or fail-secure device is one that, in the event of failure, responds in a way that will cause no harm, or at least a minimum of harm, to other devices or danger to personnel.In this slice of code a failsafe is defined.

[AP_Notify](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_Notify/AP_Notify.h) stablishes the notify flags for LEds, buzzers, alarms...

```cpp

////////////////////////////////////////////////////////////////////////////////
// Location & Navigation
////////////////////////////////////////////////////////////////////////////////
// Constants
const	float radius_of_earth 	= 6378100;	// meters


// true if we have a position estimate from AHRS
static bool have_position;

static bool rtl_complete = false;


// angle of our next navigation waypoint
static int32_t next_navigation_leg_cd;

// ground speed error in m/s
static float	groundspeed_error;
// 0-(throttle_max - throttle_cruise) : throttle nudge in Auto mode using top 1/2 of throttle stick travel
static int16_t     throttle_nudge = 0;

// receiver RSSI
static uint8_t receiver_rssi;

// the time when the last HEARTBEAT message arrived from a GCS
static uint32_t last_heartbeat_ms;

// obstacle detection information
static struct {
    // have we detected an obstacle?
    uint8_t detected_count;
    float turn_angle;
    uint16_t sonar1_distance_cm;
    uint16_t sonar2_distance_cm;

    // time when we last detected an obstacle, in milliseconds
    uint32_t detected_time_ms;
} obstacle;

// this is set to true when auto has been triggered to start
static bool auto_triggered;
...
```
Here are declared the varibles for controlling the location and the route the rover is following.This code content is, is resume, some definitions for **control variables**.

```cpp
////////////////////////////////////////////////////////////////////////////////
// Ground speed
////////////////////////////////////////////////////////////////////////////////
// The amount current ground speed is below min ground speed.  meters per second
static float 	ground_speed = 0;
static int16_t throttle_last = 0, throttle = 500;
...
```
The `ground_speed` and the `throttle` variables definition, for later, be able t control them.

```cpp

////////////////////////////////////////////////////////////////////////////////
// CH7 control
////////////////////////////////////////////////////////////////////////////////

// Used to track the CH7 toggle state.
// When CH7 goes LOW PWM from HIGH PWM, this value will have been set true
// This allows advanced functionality to know when to execute
static bool ch7_flag;

////////////////////////////////////////////////////////////////////////////////
// Battery Sensors
////////////////////////////////////////////////////////////////////////////////
static AP_BattMonitor battery;
...
```

`ch7_flag` is the channel 7 control variable.When it returns true the ch7 is in Pulse-width modulation (PWM) high state.

After the ch7 control block we find `battery`,a [AP_BattMonitor](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_BattMonitor/AP_BattMonitor.h) class variable, for battery monitoring.

```cpp

////////////////////////////////////////////////////////////////////////////////
// Navigation control variables
////////////////////////////////////////////////////////////////////////////////
// The instantaneous desired lateral acceleration in m/s/s
static float lateral_acceleration;

////////////////////////////////////////////////////////////////////////////////
// Waypoint distances
////////////////////////////////////////////////////////////////////////////////
// Distance between rover and next waypoint.  Meters
static float wp_distance;
// Distance between previous and next waypoint.  Meters
static int32_t wp_totalDistance;

////////////////////////////////////////////////////////////////////////////////
// Conditional command
////////////////////////////////////////////////////////////////////////////////
// A value used in condition commands (eg delay, change alt, etc.)
// For example in a change altitude command, it is the altitude to change to.
static int32_t 	condition_value;
// A starting value used to check the status of a conditional command.
// For example in a delay command the condition_start records that start time for the delay
static int32_t 	condition_start;

////////////////////////////////////////////////////////////////////////////////
// 3D Location vectors
// Location structure defined in AP_Common
////////////////////////////////////////////////////////////////////////////////
// The home location used for RTL.  The location is set when we first get stable GPS lock
static const struct	Location &home = ahrs.get_home();
// Flag for if we have gps lock and have set the home location
static bool	home_is_set;
// The location of the previous waypoint.  Used for track following and altitude ramp calculations
static struct 	Location prev_WP;
// The location of the current/active waypoint.  Used for track following
static struct 	Location next_WP;
// The location of the active waypoint in Guided mode.
static struct  	Location guided_WP;

////////////////////////////////////////////////////////////////////////////////
// IMU variables
////////////////////////////////////////////////////////////////////////////////
// The main loop execution time.  Seconds
//This is the time between calls to the DCM algorithm and is the Integration time for the gyros.
static float G_Dt						= 0.02;

////////////////////////////////////////////////////////////////////////////////
// Performance monitoring
////////////////////////////////////////////////////////////////////////////////
// Timer used to accrue data and trigger recording of the performanc monitoring log message
static int32_t 	perf_mon_timer;
// The maximum main loop execution time recorded in the current performance monitoring interval
static uint32_t 	G_Dt_max;

////////////////////////////////////////////////////////////////////////////////
// System Timers
////////////////////////////////////////////////////////////////////////////////
// Time in microseconds of start of main control loop.
static uint32_t 	fast_loopTimer_us;
// Number of milliseconds used in last main loop cycle
static uint32_t		delta_us_fast_loop;
// Counter of main loop executions.  Used for performance monitoring and failsafe processing
static uint16_t			mainLoop_count;

// set if we are driving backwards
static bool in_reverse;
...
```
This part of the code stablish variable for diferents aims.You can follow the comments over the code itself, in order to inquire what performance is each variable designed for.

```cpp
////////////////////////////////////////////////////////////////////////////////
// Top-level logic
////////////////////////////////////////////////////////////////////////////////

/*
  scheduler table - all regular tasks should be listed here, along
  with how often they should be called (in 20ms units) and the maximum
  time they are expected to take (in microseconds)
 */
static const AP_Scheduler::Task scheduler_tasks[] PROGMEM = {
	{ read_radio,             1,   1000 },
    { ahrs_update,            1,   6400 },
    { read_sonars,            1,   2000 },
    { update_current_mode,    1,   1500 },
    { set_servos,             1,   1500 },
    { update_GPS_50Hz,        1,   2500 },
    { update_GPS_10Hz,        5,   2500 },
    { update_alt,             5,   3400 },
    { navigate,               5,   1600 },
    { update_compass,         5,   2000 },
    { update_commands,        5,   1000 },
    { update_logging1,        5,   1000 },
    { update_logging2,        5,   1000 },
    { gcs_retry_deferred,     1,   1000 },
    { gcs_update,             1,   1700 },
    { gcs_data_stream_send,   1,   3000 },
    { read_control_switch,   15,   1000 },
    { read_trim_switch,       5,   1000 },
    { read_battery,           5,   1000 },
    { read_receiver_rssi,     5,   1000 },
    { update_events,          1,   1000 },
    { check_usb_mux,         15,   1000 },
    { mount_update,           1,    600 },
    { gcs_failsafe_check,     5,    600 },
    { compass_accumulate,     1,    900 },
    { update_notify,          1,    300 },
    { one_second_loop,       50,   3000 }
};
...
```
This is a control table, as commented on the code. It contains task and their periodicity.

```cpp

/*
  setup is called when the sketch starts
 */
void setup() {
    cliSerial = hal.console;

    // load the default values of variables listed in var_info[]
    AP_Param::setup_sketch_defaults();

    // rover does not use arming nor pre-arm checks
    AP_Notify::flags.armed = true;
    AP_Notify::flags.pre_arm_check = true;
    AP_Notify::flags.failsafe_battery = false;

    notify.init(false);

    battery.init();

    rssi_analog_source = hal.analogin->channel(ANALOG_INPUT_NONE);

	init_ardupilot();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));
}
...
```
This is the first funtion to be called.
Init the battery and the scheduler.Also, stablish a channel for Analog inputs.

```cpp
/*
  loop() is called rapidly while the sketch is running
 */
void loop()
{
    // wait for an INS sample
    if (!ins.wait_for_sample(1000)) {
        return;
    }
    uint32_t timer = hal.scheduler->micros();

    delta_us_fast_loop	= timer - fast_loopTimer_us;
    G_Dt                = delta_us_fast_loop * 1.0e-6f;
    fast_loopTimer_us   = timer;

	if (delta_us_fast_loop > G_Dt_max)
		G_Dt_max = delta_us_fast_loop;

    mainLoop_count++;

    // tell the scheduler one tick has passed
    scheduler.tick();

    scheduler.run(19500U);
}
...
```

This `loop()` calls the main loop scheduler for APM contained is [AP_scheduler](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_Scheduler/AP_Scheduler.h).

Remarkable functions called do the following:

- `micros()` belong to [AP_HAL/Scheduler](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/Scheduler.h) which change units from mili to micro.


- `tick()` is called when one tick has passed.


- `run()` run the tasks. Call this once per 'tick'.


```cpp
// update AHRS system
static void ahrs_update()
{
    ahrs.set_armed(hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED);

#if HIL_MODE != HIL_MODE_DISABLED
    // update hil before AHRS update
    gcs_update();
#endif

    // when in reverse we need to tell AHRS not to assume we are a
    // 'fly forward' vehicle, otherwise it will see a large
    // discrepancy between the mag and the GPS heading and will try to
    // correct for it, leading to a large yaw error
    ahrs.set_fly_forward(!in_reverse);

    ahrs.update();

    // if using the EKF get a speed update now (from accelerometers)
    Vector3f velocity;
    if (ahrs.get_velocity_NED(velocity)) {
        ground_speed = pythagorous2(velocity.x, velocity.y);
    }

    if (should_log(MASK_LOG_ATTITUDE_FAST))
        Log_Write_Attitude();

    if (should_log(MASK_LOG_IMU))
        DataFlash.Log_Write_IMU(ins);
}
...
```


For updating AHRS system methods from [AP_AHRS](https://github.com/BeaglePilot/ardupilot/tree/master/libraries/AP_AHRS) are called. In `AP_AHRS.h` you can find the definitions and in `AP_AHRS.cpp` the impplementations of these methods.Here are some notes about the methods:

-    `virtual bool get_relative_position_NED(Vector3f &vec) const { return false; }` returns a position relative to home in meters, North/East/Down
order. This will only be accurate if `have_inertial_nav()` is true.
```cpp
// if using the EKF get a speed update now (from accelerometers)
    Vector3f velocity;
    if (ahrs.get_velocity_NED(velocity)) {
        ground_speed = pythagorous2(velocity.x, velocity.y);
    }
    ```


- `void set_armed(bool setting) {_flags.armed = setting;}` sets the armed flag allows EKF enter static mode when disarmed.
```cpp
shrs.set_armed(hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED);
```


` Log_Write_IMU`is a logging method common to all vehicles, exported from [DataFlash](https://github.com/diydrones/ardupilot/blob/master/libraries/DataFlash/DataFlash.h#L52) library.

```cpp
/*
  update camera mount - 50Hz
 */
static void mount_update(void)
{
#if MOUNT == ENABLED
	camera_mount.update_mount_position();
#endif
#if CAMERA == ENABLED
    camera.trigger_pic_cleanup();
#endif
}

static void update_alt()
{
    barometer.read();
    if (should_log(MASK_LOG_IMU)) {
        Log_Write_Baro();
    }
}
...
```
This slice of code checks if the CAMERA and the MOUNT are enabled and then call [AP_CAMERA](https://github.com/BeaglePilot/ardupilot/tree/master/libraries/AP_Camera) AND [AP_MOUNT](https://github.com/diydrones/ardupilot/tree/master/libraries/AP_Mount) methods.

- ` update_mount_position();` should be called periodically, for knowing the actual position of the camera mount.


- `   trigger_pic_cleanup();`
de-activate the trigger after some delay, but without using a `delay()` function should be called at 50hz from main program.


- Also calls `barometer.read()`  for reading the `AP_Baro`class `barometer`variable.


```cpp
/*
  check for GCS failsafe - 10Hz
 */
static void gcs_failsafe_check(void)
{
	if (g.fs_gcs_enabled) {
        failsafe_trigger(FAILSAFE_EVENT_GCS, last_heartbeat_ms != 0 && (millis() - last_heartbeat_ms) > 2000);
    }
}
...
```
Check if there is a failsafe.

```cpp
/*
  if the compass is enabled then try to accumulate a reading
 */
static void compass_accumulate(void)
{
    if (g.compass_enabled) {
        compass.accumulate();
    }
}
...
```

The compass method are called to accumulate reading. `acucumulate` method uses spare CPU cycles to accumulate values from the compass if possible.You can find the implementeation of this method  [here](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_Compass/Compass.h#L99).
```cpp

/*
  check for new compass data - 10Hz
 */
static void update_compass(void)
{
    if (g.compass_enabled && compass.read()) {
        ahrs.set_compass(&compass);
        // update offsets
        compass.learn_offsets();
        if (should_log(MASK_LOG_COMPASS)) {
            Log_Write_Compass();
        }
    } else {
        ahrs.set_compass(NULL);
    }
}
...
```
Compass methods are used to check for new compass data.`void learn_offsets(void);` performs automatic offset updates.

```cpp
/*
  log some key data - 10Hz
 */
static void update_logging1(void)
{
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST))
        Log_Write_Attitude();

    if (should_log(MASK_LOG_CTUN))
        Log_Write_Control_Tuning();

    if (should_log(MASK_LOG_NTUN))
        Log_Write_Nav_Tuning();
}

/*
  log some key data - 10Hz
 */
static void update_logging2(void)
{
    if (should_log(MASK_LOG_STEERING)) {
        if (control_mode == STEERING || control_mode == AUTO || control_mode == RTL || control_mode == GUIDED) {
            Log_Write_Steering();
        }
    }

    if (should_log(MASK_LOG_RC))
        Log_Write_RC();
}

...
```
Here the `control_mode`and the `mask` are set up.

```cpp

/*
  update aux servo mappings
 */
static void update_aux(void)
{
    RC_Channel_aux::enable_aux_servos();

#if MOUNT == ENABLED
    camera_mount.update_mount_type();
#endif
}
...
```
This function updates the servomechanism type through `update_mount_type()` which auto-detect the mount gimbal type depending on the functions assigned .

```cpp
/*
  once a second events
 */
static void one_second_loop(void)
{
	if (should_log(MASK_LOG_CURRENT))
		Log_Write_Current();
	// send a heartbeat
	gcs_send_message(MSG_HEARTBEAT);

    // allow orientation change at runtime to aid config
    ahrs.set_orientation();

    set_control_channels();

    // cope with changes to aux functions
    update_aux();

#if MOUNT == ENABLED
    camera_mount.update_mount_type();
#endif

    // cope with changes to mavlink system ID
    mavlink_system.sysid = g.sysid_this_mav;

    static uint8_t counter;

    counter++;

    // write perf data every 20s
    if (counter % 10 == 0) {
        if (scheduler.debug() != 0) {
            hal.console->printf_P(PSTR("G_Dt_max=%lu\n"), (unsigned long)G_Dt_max);
        }
        if (should_log(MASK_LOG_PM))
            Log_Write_Performance();
        G_Dt_max = 0;
        resetPerfData();
    }

    // save compass offsets once a minute
    if (counter >= 60) {
        if (g.compass_enabled) {
            compass.save_offsets();
        }
        counter = 0;
    }
}

static void update_GPS_50Hz(void)
{
    static uint32_t last_gps_reading[GPS_MAX_INSTANCES];
	gps.update();

    for (uint8_t i=0; i<gps.num_sensors(); i++) {
        if (gps.last_message_time_ms(i) != last_gps_reading[i]) {
            last_gps_reading[i] = gps.last_message_time_ms(i);
            if (should_log(MASK_LOG_GPS)) {
                DataFlash.Log_Write_GPS(gps, i, current_loc.alt);
            }
        }
    }
}
...
```
