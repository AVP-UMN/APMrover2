# Log.pde


The [Log.pde](https://github.com/BeaglePilot/ardupilot/blob/master/APMrover2/Log.pde) file contains functions for writing and reading packets from DataFlash log memory.Also, allows the user to dump or erase logs.
```cpp
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if LOGGING_ENABLED == ENABLED

// Code to Write and Read packets from DataFlash log memory
// Code to interact with the user to dump or erase logs
...
```
Check if `LOGGING_ENABLE`is already working.
```cpp
// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
static int8_t	dump_log(uint8_t argc, 			const Menu::arg *argv);
static int8_t	erase_logs(uint8_t argc, 		const Menu::arg *argv);
static int8_t	select_logs(uint8_t argc, 		const Menu::arg *argv);
...
```
 Here you can find `dump_log`, `erase_log`and `select_logs` functions definitions.They are defined here to let the Menu be constructed.
 ```cpp
 // Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Common for implementation details
static const struct Menu::command log_menu_commands[] PROGMEM = {
	{"dump",	dump_log},
	{"erase",	erase_logs},
	{"enable",	select_logs},
	{"disable",	select_logs}
};
...
```
Here the menu is created as a array of structs.The strings between quotes represent the options in the menu.In [AP_Common](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_Common/AP_Common.h) library you will find more about menu class.

```cpp
// A Macro to create the Menu
MENU2(log_menu, "Log", log_menu_commands, print_log_menu);

static bool
print_log_menu(void)
{
	cliSerial->printf_P(PSTR("logs enabled: "));

	if (0 == g.log_bitmask) {
		cliSerial->printf_P(PSTR("none"));
	}else{
		// Macro to make the following code a bit easier on the eye.
		// Pass it the capitalised name of the log option, as defined
		// in defines.h but without the LOG_ prefix.  It will check for
		// the bit being set and print the name of the log option to suit.
		#define PLOG(_s)	if (g.log_bitmask & MASK_LOG_ ## _s) cliSerial->printf_P(PSTR(" %S"), PSTR(#_s))
		PLOG(ATTITUDE_FAST);
		PLOG(ATTITUDE_MED);
		PLOG(GPS);
		PLOG(PM);
		PLOG(CTUN);
		PLOG(NTUN);
		PLOG(MODE);
		PLOG(IMU);
		PLOG(CMD);
		PLOG(CURRENT);
		PLOG(SONAR);
		PLOG(COMPASS);
		PLOG(CAMERA);
		PLOG(STEERING);
		#undef PLOG
	}

	cliSerial->println();

    DataFlash.ListAvailableLogs(cliSerial);
	return(true);
}
...
```
This slice of code creates a macro for the menu.A **macro** (short for "macroinstruction")  is a rule or pattern that specifies how a certain input sequence (often a sequence of characters) should be mapped to a replacement output sequence (also often a sequence of characters) according to a defined procedure.

```cpp

static int8_t
dump_log(uint8_t argc, const Menu::arg *argv)
{
    int16_t dump_log;
    uint16_t dump_log_start;
    uint16_t dump_log_end;
    uint16_t last_log_num;

    // check that the requested log number can be read
    dump_log = argv[1].i;
    last_log_num = DataFlash.find_last_log();

    if (dump_log == -2) {
        DataFlash.DumpPageInfo(cliSerial);
        return(-1);
    } else if (dump_log <= 0) {
        cliSerial->printf_P(PSTR("dumping all\n"));
        Log_Read(0, 1, 0);
        return(-1);
    } else if ((argc != 2)
               || ((uint16_t)dump_log > last_log_num))
    {
        cliSerial->printf_P(PSTR("bad log number\n"));
        return(-1);
    }

    DataFlash.get_log_boundaries(dump_log, dump_log_start, dump_log_end);
    Log_Read((uint16_t)dump_log, dump_log_start, dump_log_end);
    return 0;
}

```
This slice of code  checks if the value of `dump_log=argc[1]` is equal to -2, in that case dumps the information in the page; if not checks if the value is under 0. When the value is under 0, reads each log. Lastly, check if `argc` is not 2 or the `dump_log`variable value is greater than the `_last_log_num` for printing a error message.

```cpp
static void do_erase_logs(void)
{
	cliSerial->printf_P(PSTR("\nErasing log...\n"));
    DataFlash.EraseAll();
	cliSerial->printf_P(PSTR("\nLog erased.\n"));
}

```
This function call `EraseAll()`funtion defined in Dataflash module, for deleting the logs.
``` cpp
static int8_t
erase_logs(uint8_t argc, const Menu::arg *argv)
{
    in_mavlink_delay = true;
    do_erase_logs();
    in_mavlink_delay = false;
    return 0;
}
...
```
This funtion calls the previously defined function `do_erase_logs`in case the mavlink messages are dalayed (`in_mavlink_delay` value is true).

```cpp
static int8_t
select_logs(uint8_t argc, const Menu::arg *argv)
{
	uint16_t	bits;

	if (argc != 2) {
		cliSerial->printf_P(PSTR("missing log type\n"));
		return(-1);
	}

	bits = 0;

	// Macro to make the following code a bit easier on the eye.
	// Pass it the capitalised name of the log option, as defined
	// in defines.h but without the LOG_ prefix.  It will check for
	// that name as the argument to the command, and set the bit in
	// bits accordingly.
	//
	if (!strcasecmp_P(argv[1].str, PSTR("all"))) {
		bits = ~0;
	} else {
		#define TARG(_s)	if (!strcasecmp_P(argv[1].str, PSTR(#_s))) bits |= MASK_LOG_ ## _s
		TARG(ATTITUDE_FAST);
		TARG(ATTITUDE_MED);
		TARG(GPS);
		TARG(PM);
		TARG(CTUN);
		TARG(NTUN);
		TARG(MODE);
		TARG(IMU);
		TARG(CMD);
		TARG(CURRENT);
		TARG(SONAR);
		TARG(COMPASS);
		TARG(CAMERA);
		TARG(STEERING);
		#undef TARG
	}

	if (!strcasecmp_P(argv[0].str, PSTR("enable"))) {
		g.log_bitmask.set_and_save(g.log_bitmask | bits);
	}else{
		g.log_bitmask.set_and_save(g.log_bitmask & ~bits);
	}
	return(0);
}
...
```
 This function first check if the value of `argc` isdifferent from 2, in that case prints a missing log type.After that calls the macro defined above. Then checks if `PSTR` is enabled and in that case set and save the logs.

``` cpp

process_logs(uint8_t argc, const Menu::arg *argv)
{
	log_menu.run();
	return 0;
}
...

```
`process_logs` funtion runs the menu defined above.

```cpp
struct PACKED log_Performance {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    uint32_t loop_time;
    uint16_t main_loop_count;
    uint32_t g_dt_max;
    int16_t  gyro_drift_x;
    int16_t  gyro_drift_y;
    int16_t  gyro_drift_z;
    uint8_t  i2c_lockup_count;
    uint16_t ins_error_count;
};
...
```

This struct contains definitions of variables that will be used when working with logs.

```cpp
// Write a performance monitoring packet. Total length : 19 bytes
static void Log_Write_Performance()
{
    struct log_Performance pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PERFORMANCE_MSG),
        time_ms         : millis(),
        loop_time       : millis()- perf_mon_timer,
        main_loop_count : mainLoop_count,
        g_dt_max        : G_Dt_max,
        gyro_drift_x    : (int16_t)(ahrs.get_gyro_drift().x * 1000),
        gyro_drift_y    : (int16_t)(ahrs.get_gyro_drift().y * 1000),
        gyro_drift_z    : (int16_t)(ahrs.get_gyro_drift().z * 1000),
        i2c_lockup_count: hal.i2c->lockup_count(),
        ins_error_count  : ins.error_count()
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}
...
```
The function  `Log_Write_Performance()`contains the implementations of the `Log_Performance` struct defined above.For example the function `time_ms` should call another function `millis` which change units from micro to mili seconds(As comented in other sections).

```cpp

// Write a mission command. Total length : 36 bytes
static void Log_Write_Cmd(const AP_Mission::Mission_Command &cmd)
{
    mavlink_mission_item_t mav_cmd = {};
    AP_Mission::mission_cmd_to_mavlink(cmd,mav_cmd);
    DataFlash.Log_Write_MavCmd(mission.num_commands(),mav_cmd);
}
...
```

This functions defines a [AP_Mission Class](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_Mission/AP_Mission.h#L43) function and after that writes the logs using Dataflah function [Log_Write_MavCMd](https://github.com/diydrones/ardupilot/blob/master/libraries/DataFlash/DataFlash.h#L69), defines as: `  void Log_Write_MavCmd(uint16_t cmd_total, const mavlink_mission_item_t& mav_cmd);`
```cpp
struct PACKED log_Steering {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    float demanded_accel;
    float achieved_accel;
};
...
```
Here `Log_Steering` struct is defined.

```cpp
// Write a steering packet
static void Log_Write_Steering()
{
    struct log_Steering pkt = {
        LOG_PACKET_HEADER_INIT(LOG_STEERING_MSG),
        time_ms        : hal.scheduler->millis(),
        demanded_accel : lateral_acceleration,
        achieved_accel : gps.ground_speed() * ins.get_gyro().z,
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}
...
```
This slice of code, first defines a packet called `Log_Steering` containing implementations of functions. Then calls ` WriteBlock` Datafhash function  for writing the info in the packet.

```cpp

struct PACKED log_Startup {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    uint8_t startup_type;
    uint16_t command_total;
};
...
```

`Log_Startup` struc is defined here.This struc will be use in the next function implementation.

```cpp
static void Log_Write_Startup(uint8_t type)
{
    struct log_Startup pkt = {
        LOG_PACKET_HEADER_INIT(LOG_STARTUP_MSG),
        time_ms         : millis(),
        startup_type    : type,
        command_total   : mission.num_commands()
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));

    // write all commands to the dataflash as well
    AP_Mission::Mission_Command cmd;
    for (uint16_t i = 0; i < mission.num_commands(); i++) {
        if(mission.read_cmd_from_storage(i,cmd)) {
            Log_Write_Cmd(cmd);
        }
    }
}
...
```

`Log_Write_Startup` packet is implemented and `WriteBlock stores the data, in the same way as int he previous function.
After this all the miision commands are copied to the dataflash.The process is simply: while i is under the mission commands number, read from storage the command and write it on the dataflash.

```cpp

struct PACKED log_Control_Tuning {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t steer_out;
    int16_t roll;
    int16_t pitch;
    int16_t throttle_out;
    float accel_y;
};
...
```
Here the `log_control_Tuning` struct is defined.

https://github.com/BeaglePilot/ardupilot/blob/master/APMrover2/Log.pde#L260
