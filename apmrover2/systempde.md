# System.pde

The link to the code: [system.pde](https://github.com/diydrones/ardupilot/blob/master/APMrover2/system.pde).
This file contains funtions for the main control actions.

```cpp
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*****************************************************************************
The init_ardupilot function processes everything we need for an in - air restart
	We will determine later if we are actually on the ground and process a
	ground start in that case.

*****************************************************************************/

#if CLI_ENABLED == ENABLED

// Functions called from the top-level menu
static int8_t	process_logs(uint8_t argc, const Menu::arg *argv);	// in Log.pde
static int8_t	setup_mode(uint8_t argc, const Menu::arg *argv);	// in setup.pde
static int8_t	test_mode(uint8_t argc, const Menu::arg *argv);		// in test.cpp
static int8_t   reboot_board(uint8_t argc, const Menu::arg *argv);
...
```

Enables the Cli and definessome funtions that can be called from the main, as Menu class.

```cpp
// This is the help function
// PSTR is an AVR macro to read strings from flash memory
// printf_P is a version of print_f that reads from flash memory
static int8_t	main_menu_help(uint8_t argc, const Menu::arg *argv)
{
	cliSerial->printf_P(PSTR("Commands:\n"
						 "  logs        log readback/setup mode\n"
						 "  setup       setup mode\n"
						 "  test        test mode\n"
						 "\n"
						 "Move the slide switch and reset to FLY.\n"
						 "\n"));
	return(0);
}
...
```
This function defines a macro for the menu, which presents the posible options.

```cpp
// Command/function table for the top-level menu.
static const struct Menu::command main_menu_commands[] PROGMEM = {
//   command		function called
//   =======        ===============
	{"logs",		process_logs},
	{"setup",		setup_mode},
	{"test",		test_mode},
    {"reboot",      reboot_board},
	{"help",		main_menu_help}
};
...
```
This slice of code presents a table for the menu options.

```cpp
// Create the top-level menu object.
MENU(main_menu, THISFIRMWARE, main_menu_commands);

static int8_t reboot_board(uint8_t argc, const Menu::arg *argv)
{
    hal.scheduler->reboot(false);
    return 0;
}

...
```
Here the main `MENU` is implementes and  the schedules [reboot](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/Scheduler.h#L46) funtion is disabled.

```cpp
// the user wants the CLI. It never exits
static void run_cli(AP_HAL::UARTDriver *port)
{
    // disable the failsafe code in the CLI
    hal.scheduler->register_timer_failsafe(NULL,1);

    // disable the mavlink delay callback
    hal.scheduler->register_delay_callback(NULL, 5);

    cliSerial = port;
    Menu::set_port(port);
    port->set_blocking_writes(true);

    while (1) {
        main_menu.run();
    }
}

#endif // CLI_ENABLED
...
```
This slice of code ensures to have the ClI enabled, updating the values of the necessary funtions.

```cpp
static void init_ardupilot()
{
...
```
ìnit_ardupilot`can be considered the most important function over this file.
	// Console serial port
	//This function preocesses everything needed to have the robot Ready to Drive.

```cpp
	// The console port buffers are defined to be sufficiently large to support
	// the console's use as a logging device, optionally as the GPS port when
	// GPS_PROTOCOL_IMU is selected, and as the telemetry port.
	//
	// XXX This could be optimised to reduce the buffer sizes in the cases
	// where they are not otherwise required.
	//
    hal.uartA->begin(SERIAL0_BAUD, 128, 128);

	// GPS serial port.
	//
	// XXX currently the EM406 (SiRF receiver) is nominally configured
	// at 57600, however it's not been supported to date.  We should
	// probably standardise on 38400.
	//
	// XXX the 128 byte receive buffer may be too small for NMEA, depending
	// on the message set configured.
	//
    // standard gps running
    hal.uartB->begin(38400, 256, 16);
...
```
Here you can fnd the specifications needed.
```cppp
#if GPS2_ENABLE
    if (hal.uartE != NULL) {
        hal.uartE->begin(38400, 256, 16);
    }
#endif
...
```
Checks if the GPS2 is enabled and if yes enable the uartE devide.


```cpp
	cliSerial->printf_P(PSTR("\n\nInit " FIRMWARE_STRING
						 "\n\nFree RAM: %u\n"),
                        hal.util->available_memory());

	//
	// Check the EEPROM format version before loading any parameters from EEPROM.
	//
...
```
Cehcks the EEPROM status for loading parameters later.

```cpp
    load_parameters();

    BoardConfig.init();

    ServoRelayEvents.set_channel_mask(0xFFF0);

    set_control_channels();
...
```
Set the parameters, the board configuration and some others.

```cpp
    // after parameter load setup correct baud rate on uartA
    hal.uartA->begin(map_baudrate(g.serial0_baud));

    // keep a record of how many resets have happened. This can be
    // used to detect in-flight resets
    g.num_resets.set_and_save(g.num_resets+1);
...
```
Here takes care of the number of reboots.

```cpp
    // init baro before we start the GCS, so that the CLI baro test works
    barometer.init();

	// init the GCS
	gcs[0].init(hal.uartA);

    // we start by assuming USB connected, as we initialed the serial
    // port with SERIAL0_BAUD. check_usb_mux() fixes this if need be.
    usb_connected = true;
    check_usb_mux();

    // we have a 2nd serial port for telemetry
    gcs[1].setup_uart(hal.uartC, map_baudrate(g.serial1_baud), 128, 128);

...
```
Confgure and enable all ports and devices.

```cpp
#if MAVLINK_COMM_NUM_BUFFERS > 2
    if (g.serial2_protocol == SERIAL2_FRSKY_DPORT ||
        g.serial2_protocol == SERIAL2_FRSKY_SPORT) {
        frsky_telemetry.init(hal.uartD, g.serial2_protocol);
    } else {
        gcs[2].setup_uart(hal.uartD, map_baudrate(g.serial2_baud), 128, 128);
    }
#endif

	mavlink_system.sysid = g.sysid_this_mav;
	...
	```
This slice of code stablish the MAVLINK protocol status.
```cpp

#if LOGGING_ENABLED == ENABLED
	DataFlash.Init(log_structure, sizeof(log_structure)/sizeof(log_structure[0]));
    if (!DataFlash.CardInserted()) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("No dataflash card inserted"));
        g.log_bitmask.set(0);
    } else if (DataFlash.NeedErase()) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("ERASING LOGS"));
		do_erase_logs();
    }
	if (g.log_bitmask != 0) {
		start_logging();
	}
#endif
...
```
Here the logs recording is initialized.
```cpp

    // Register mavlink_delay_cb, which will run anytime you have
    // more than 5ms remaining in your call to hal.scheduler->delay
    hal.scheduler->register_delay_callback(mavlink_delay_cb, 5);
...
```
This slice of code register the delay status.

```cpp
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
    apm1_adc.Init();      // APM ADC library initialization
#endif

	if (g.compass_enabled==true) {
		if (!compass.init()|| !compass.read()) {
            cliSerial->println_P(PSTR("Compass initialisation failed!"));
            g.compass_enabled = false;
        } else {
            ahrs.set_compass(&compass);
            //compass.get_offsets();						// load offsets to account for airframe magnetic interference
        }
	}

...
```
If the board is the correct one, the compass data set ups are done.

```cpp
// initialise sonar
    init_sonar();

    // and baro for EKF
    init_barometer();

	// Do GPS init
	gps.init(&DataFlash);
...
```
Here initialized the sonar, the GPs and the barometer.

```cpp

	//mavlink_system.sysid = MAV_SYSTEM_ID;				// Using g.sysid_this_mav
	mavlink_system.compid = 1;	//MAV_COMP_ID_IMU;   // We do not check for comp id
	mavlink_system.type = MAV_TYPE_GROUND_ROVER;

...
```
This slice of code take care of some `mavlink_system` variables.

```cpp
    rc_override_active = hal.rcin->set_overrides(rc_override, 8);

	init_rc_in();		// sets up rc channels from radio
	init_rc_out();		// sets up the timer libs

    relay.init();
...
```
The RC I/O are initialized.

```cpp
/*
      setup the 'main loop is dead' check. Note that this relies on
      the RC library being initialised.
     */
    hal.scheduler->register_timer_failsafe(failsafe_check, 1000);

	// If the switch is in 'menu' mode, run the main menu.
	//
	// Since we can't be sure that the setup or test mode won't leave
	// the system in an odd state, we don't let the user exit the top
	// menu; they must reset in order to fly.
	//
    const prog_char_t *msg = PSTR("\nPress ENTER 3 times to start interactive setup\n");
    cliSerial->println_P(msg);
    if (gcs[1].initialised) {
        hal.uartC->println_P(msg);
    }
    ...
    ```
the main loop begins here.With three enters (which mean `num_gcs`>2) the interactive mode is initialized.  ```cpp
    if (num_gcs > 2 && gcs[2].initialised) {
        hal.uartD->println_P(msg);
    }

	startup_ground();

	if (should_log(MASK_LOG_CMD)) {
        Log_Write_Startup(TYPE_GROUNDSTART_MSG);
    }

    set_mode((enum mode)g.initial_mode.get());

	// set the correct flight mode
	// ---------------------------
	reset_control_switch();
}
...
```
The contol mode is set, by using the switch.

https://github.com/diydrones/ardupilot/blob/master/APMrover2/system.pde#L236
