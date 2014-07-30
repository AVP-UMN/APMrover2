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
The [AP_InertialSensor](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_InertialSensor/AP_InertialSensor.h) is an abstraction for gyro and accelerometer measurements which are correctly aligned to the body axes and scaled to SI units.In the `AP_InertialSensor.h` code we find the `Sample_rate` defintion.
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
Defines the compass using  [AP_Compass.h](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_Compass/AP_Compass.h#L6) which catchs-all header that defines all supported compass classes.

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

Defines [AP_Mission](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_Mission/AP_Mission.h) methods. `AP_Mission` library performances are:

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
 Here the radio- control parameters are initialised.

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
