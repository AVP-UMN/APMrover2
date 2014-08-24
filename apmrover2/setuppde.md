# setup.pde

The [setup.pde](https://github.com/diydrones/ardupilot/blob/master/APMrover2/setup.pde) file includes function and definitions for stablishing all the neccesary things to make the autopilot work.

```cpp
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if CLI_ENABLED == ENABLED
...
```
Here starts the file, with the enablement of the cli.
```cpp

// Functions called from the setup menu
static int8_t	setup_radio			(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_show			(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_factory		(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_flightmodes	(uint8_t argc, const Menu::arg *argv);
#if !defined( __AVR_ATmega1280__ )
static int8_t   setup_accel_scale   (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_set           (uint8_t argc, const Menu::arg *argv);
#endif
static int8_t   setup_level         (uint8_t argc, const Menu::arg *argv);
static int8_t	setup_erase			(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_compass		(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_declination	(uint8_t argc, const Menu::arg *argv);
...
```
Here, you can find the definitions of the possibilities showed at the setup menu.

```cpp
// Command/function table for the setup menu
static const struct Menu::command setup_menu_commands[] PROGMEM = {
	// command			function called
	// =======        	===============
	{"reset", 			setup_factory},
	{"radio",			setup_radio},
	{"modes",			setup_flightmodes},
	{"level",			setup_level},
#if !defined( __AVR_ATmega1280__ )
    {"accel",           setup_accel_scale},
#endif
	{"compass",			setup_compass},
	{"declination",		setup_declination},
	{"show",			setup_show},
#if !defined( __AVR_ATmega1280__ )
	{"set",				setup_set},
#endif
	{"erase",			setup_erase},
};
...
```
This slice of code stablish the setup menu display.
```cpp

// Create the setup menu object.
MENU(setup_menu, "setup", setup_menu_commands);

// Called from the top-level menu to run the setup menu.
static int8_t
setup_mode(uint8_t argc, const Menu::arg *argv)
{
	// Give the user some guidance
	cliSerial->printf_P(PSTR("Setup Mode\n"
						 "\n"
						 "IMPORTANT: if you have not previously set this system up, use the\n"
						 "'reset' command to initialize the EEPROM to sensible default values\n"
						 "and then the 'radio' command to configure for your radio.\n"
						 "\n"));

	// Run the setup menu.  When the menu exits, we will return to the main menu.
	setup_menu.run();
    return 0;
}
...
```
This slice of code stablish the object MENU, and the procedure to call it from the main.

```cpp
/ Print the current configuration.
// Called by the setup menu 'show' command.
static int8_t
setup_show(uint8_t argc, const Menu::arg *argv)
{
...
```
`setup_show`function  print the current configuration, and it is called by the "show" command, as you can see above.

``` cpp
#if !defined( __AVR_ATmega1280__ )
	AP_Param *param;
	ap_var_type type;

	//If a parameter name is given as an argument to show, print only that parameter
	if(argc>=2)
	{

		param=AP_Param::find(argv[1].str, &type);

		if(!param)
		{
			cliSerial->printf_P(PSTR("Parameter not found: '%s'\n"), argv[1]);
		return 0;
		}

...
```
If the "show" command is accompained but another one, only prints that parameter configuration or a message if it can't be found.

```cpp

		//Print differently for different types, and include parameter type in output.
		switch (type) {
			case AP_PARAM_INT8:
				cliSerial->printf_P(PSTR("INT8  %s: %d\n"), argv[1].str, (int)((AP_Int8 *)param)->get());
				break;
			case AP_PARAM_INT16:
				cliSerial->printf_P(PSTR("INT16 %s: %d\n"), argv[1].str, (int)((AP_Int16 *)param)->get());
				break;
			case AP_PARAM_INT32:
				cliSerial->printf_P(PSTR("INT32 %s: %ld\n"), argv[1].str, (long)((AP_Int32 *)param)->get());
				break;
			case AP_PARAM_FLOAT:
				cliSerial->printf_P(PSTR("FLOAT %s: %f\n"), argv[1].str, ((AP_Float *)param)->get());
				break;
			default:
				cliSerial->printf_P(PSTR("Unhandled parameter type for %s: %d.\n"), argv[1].str, type);
				break;
		}

		return 0;
	}

...
```
Here is stablished what should be printed for each parameter.

```cpp
#endif

	// clear the area
	print_blanks(8);

	report_radio();
	report_batt_monitor();
	report_gains();
	report_throttle();
	report_modes();
	report_compass();

	cliSerial->printf_P(PSTR("Raw Values\n"));
	print_divider();

...
```
This slice of code, clear the screen and repots some values.
```cpp
    AP_Param::show_all(cliSerial);

	return(0);
}
...
```
The function finished calling [shown_all](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_Param/AP_Param.cpp#L1074) from AP_Param module, that print the value of all variables.
```cpp
#if !defined( __AVR_ATmega1280__ )

//Set a parameter to a specified value. It will cast the value to the current type of the
//parameter and make sure it fits in case of INT8 and INT16
static int8_t setup_set(uint8_t argc, const Menu::arg *argv)
{
	int8_t value_int8;
	int16_t value_int16;

	AP_Param *param;
	enum ap_var_type p_type;

	if(argc!=3)
	{
		cliSerial->printf_P(PSTR("Invalid command. Usage: set <name> <value>\n"));
		return 0;
	}

...
```
This function set a parameter to a specified value, as a third parameter, so first checks if there are three input commands.

```cpp
	param = AP_Param::find(argv[1].str, &p_type);
	if(!param)
	{
		cliSerial->printf_P(PSTR("Param not found: %s\n"), argv[1].str);
		return 0;
	}
...
```
Find the param or sent a 'not found' message.

```cpp
	switch(p_type)
	{
		case AP_PARAM_INT8:
			value_int8 = (int8_t)(argv[2].i);
			if(argv[2].i!=value_int8)
			{
				cliSerial->printf_P(PSTR("Value out of range for type INT8\n"));
				return 0;
			}
			((AP_Int8*)param)->set_and_save(value_int8);
			break;
		case AP_PARAM_INT16:
			value_int16 = (int16_t)(argv[2].i);
			if(argv[2].i!=value_int16)
			{
				cliSerial->printf_P(PSTR("Value out of range for type INT16\n"));
				return 0;
			}
			((AP_Int16*)param)->set_and_save(value_int16);
			break;

		//int32 and float don't need bounds checking, just use the value provoded by Menu::arg
		case AP_PARAM_INT32:
			((AP_Int32*)param)->set_and_save(argv[2].i);
			break;
		case AP_PARAM_FLOAT:
			((AP_Float*)param)->set_and_save(argv[2].f);
			break;
		default:
			cliSerial->printf_P(PSTR("Cannot set parameter of type %d.\n"), p_type);
			break;
	}

	return 0;
}
#endif

Enters in a case, where depending of the input parameter it is set in one way or another.

```cpp
// Initialise the EEPROM to 'factory' settings (mostly defined in APM_Config.h or via defaults).
// Called by the setup menu 'factoryreset' command.
static int8_t
setup_factory(uint8_t argc, const Menu::arg *argv)

...
```
This function is called by the "factoryreset" command and retrieve the initial configuration (factory config).

```cpp
{
	int			c;

	cliSerial->printf_P(PSTR("\nType 'Y' and hit Enter to perform factory reset, any other key to abort: "));
...
```
Ask for confirming that you want to reset values.

```cpp
	do {
		c = cliSerial->read();
	} while (-1 == c);

	if (('y' != c) && ('Y' != c))
		return(-1);
	AP_Param::erase_all();
	cliSerial->printf_P(PSTR("\nFACTORY RESET complete - please reset board to continue"));

	for (;;) {
	}
	// note, cannot actually return here
	return(0);
}

...
```
Calls `erase_all()`function from `AP_Param`module for restarting all the parameters.

```cpp

// Perform radio setup.
// Called by the setup menu 'radio' command.
static int8_t
setup_radio(uint8_t argc, const Menu::arg *argv)
{
...
```
This function is called using the "radio" command and set the `rc_channels`parameters.
```cpp
	cliSerial->printf_P(PSTR("\n\nRadio Setup:\n"));
	uint8_t i;

	for(i = 0; i < 100;i++){
		delay(20);
		read_radio();
	}


	if(channel_steer->radio_in < 500){
		while(1){
			cliSerial->printf_P(PSTR("\nNo radio; Check connectors."));
			delay(1000);
			// stop here
		}
	}
...
```
Delays the radio input and check if the radio_in are under 500, if not ask for checking the connectors.

```cpp

	channel_steer->radio_min 		= channel_steer->radio_in;
	channel_throttle->radio_min 	= channel_throttle->radio_in;
	g.rc_2.radio_min = g.rc_2.radio_in;
	g.rc_4.radio_min = g.rc_4.radio_in;
	g.rc_5.radio_min = g.rc_5.radio_in;
	g.rc_6.radio_min = g.rc_6.radio_in;
	g.rc_7.radio_min = g.rc_7.radio_in;
	g.rc_8.radio_min = g.rc_8.radio_in;
...
```
Stablish the minimun radio input for Throttle and steer channels.
```cpp
	channel_steer->radio_max 		= channel_steer->radio_in;
	channel_throttle->radio_max 	= channel_throttle->radio_in;
	g.rc_2.radio_max = g.rc_2.radio_in;
	g.rc_4.radio_max = g.rc_4.radio_in;
	g.rc_5.radio_max = g.rc_5.radio_in;
	g.rc_6.radio_max = g.rc_6.radio_in;
	g.rc_7.radio_max = g.rc_7.radio_in;
	g.rc_8.radio_max = g.rc_8.radio_in;
	...
	```
The same for the maximun values.
```cpp

	channel_steer->radio_trim 		= channel_steer->radio_in;
	g.rc_2.radio_trim = 1500;
	g.rc_4.radio_trim = 1500;
	g.rc_5.radio_trim = 1500;
	g.rc_6.radio_trim = 1500;
	g.rc_7.radio_trim = 1500;
	g.rc_8.radio_trim = 1500;
...
```
Also the channel steer trim is stablished to 1500.
```cpp
	cliSerial->printf_P(PSTR("\nMove all controls to each extreme. Hit Enter to save: \n"));
	while(1){

		delay(20);
		// Filters radio input - adjust filters in the radio.pde file
		// ----------------------------------------------------------
		read_radio();

		channel_steer->update_min_max();
		channel_throttle->update_min_max();
		g.rc_2.update_min_max();
		g.rc_4.update_min_max();
		g.rc_5.update_min_max();
		g.rc_6.update_min_max();
		g.rc_7.update_min_max();
		g.rc_8.update_min_max();

		if(cliSerial->available() > 0){
            while (cliSerial->available() > 0) {
                cliSerial->read();
            }
			channel_steer->save_eeprom();
			channel_throttle->save_eeprom();
			g.rc_2.save_eeprom();
			g.rc_4.save_eeprom();
			g.rc_5.save_eeprom();
			g.rc_6.save_eeprom();
			g.rc_7.save_eeprom();
			g.rc_8.save_eeprom();
			print_done();
			break;
		}
	}
	trim_radio();
	report_radio();
	return(0);
}

...
```
All the changes are updated and saved.

```cpp
static int8_t
setup_flightmodes(uint8_t argc, const Menu::arg *argv)
{
...
```
This functions sets the posible control modes, when driving.

```cpp
	uint8_t switchPosition, mode = 0;

	cliSerial->printf_P(PSTR("\nMove RC toggle switch to each position to edit, move aileron stick to select modes."));
	print_hit_enter();
	trim_radio();

	while(1){
		delay(20);
		read_radio();
		switchPosition = readSwitch();
...
```
This slice of code reads the position of the switch, for the input mode.

```cpp
		// look for control switch change
		if (oldSwitchPosition != switchPosition){
			// force position 5 to MANUAL
			if (switchPosition > 4) {
				modes[switchPosition] = MANUAL;
			}
			// update our current mode
			mode = modes[switchPosition];

			// update the user
			print_switch(switchPosition, mode);

			// Remember switch position
			oldSwitchPosition = switchPosition;
		}
	...
	```
	Checks for a new switch position and updated the control mode.

```cpp

		// look for stick input
		int radioInputSwitch = radio_input_switch();

		if (radioInputSwitch != 0){

			mode += radioInputSwitch;

			while (
				mode != MANUAL &&
				mode != HOLD &&
				mode != LEARNING &&
				mode != STEERING &&
				mode != AUTO &&
				mode != RTL)
			{
				if (mode < MANUAL)
					mode = RTL;
				else if (mode > RTL)
					mode = MANUAL;
				else
					mode += radioInputSwitch;
			}
	...
	```
	After reading the radio input the mode is changed.


```cpp
			// Override position 5
			if(switchPosition > 4)
				mode = MANUAL;

			// save new mode
			modes[switchPosition] = mode;

			// print new mode
			print_switch(switchPosition, mode);
		}
...
```
The new mode is saved and printed for the user to know it.
```cpp

		// escape hatch
		if(cliSerial->available() > 0){
		    // save changes
            for (mode=0; mode<6; mode++)
                modes[mode].save();
			report_modes();
			print_done();
			return (0);
		}
	}
}
...
```

This slice of code is for getting out of the mode selection.

```cpp

static int8_t
setup_declination(uint8_t argc, const Menu::arg *argv)
{
	compass.set_declination(radians(argv[1].f));
	report_compass();
    return 0;
}
...
```
The `setup_declination`funtion reports the compass data.

```cpp

static int8_t
setup_erase(uint8_t argc, const Menu::arg *argv)
{
	int			c;

	cliSerial->printf_P(PSTR("\nType 'Y' and hit Enter to erase all waypoint and parameter data, any other key to abort: "));

	do {
		c = cliSerial->read();
	} while (-1 == c);

	if (('y' != c) && ('Y' != c))
		return(-1);
	zero_eeprom();
	return 0;
}
...
```

This slice of code implements the `setup_erase`function. This function erase the EEPROM memory where the wp and parameters are stored.

https://github.com/diydrones/ardupilot/blob/master/APMrover2/setup.pde#L401
