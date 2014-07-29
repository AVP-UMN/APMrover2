# APMrover2.pde

Link to the code: [APMrover2.pde](https://github.com/BeaglePilot/ardupilot/blob/master/APMrover2/APMrover2.pde).

---
The **firmware** is a block of machine instructions for specific purposes, engraved in memory, typically read / write (ROM, EEPROM, flash, etc..), Which provides the lowest level of logic that controls the electronic circuits of a device of any kind. It is tightly integrated with the electronic device being software that has direct interaction with the hardware: the manager with remote commands to execute properly.
In summary, a **firmware is software that manages the hardware physically**.

---

In this file is the firmeware of `APMrover2`. All   variables, struct and functions needed are defined and implemented here:



```cpp
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define THISFIRMWARE "ArduRover v2.46beta2"
...
```
- Here a firmware is defined with "ArduRover v2.46beta2" name.

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
```
- This slice of code provides a description of the content and license, see full text in the `APMrover2.pde`file.


- The channels state / function is declared in the commet lines.

```cpp
...

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

#include <AP_NavEKF.h>  //Altitude and position estimation.

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
- This code has been modified for adding elucidative notes beside the headers importations.

https://github.com/BeaglePilot/ardupilot/blob/master/APMrover2/APMrover2.pde#L123
