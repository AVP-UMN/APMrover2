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
