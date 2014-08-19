# commands_logic.pde

Link to the code: [commands_logic.pde](https://github.com/BeaglePilot/ardupilot/blob/master/APMrover2/commands_logic.pde)

```cpp
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// forward declarations to make compiler happy
static void do_nav_wp(const AP_Mission::Mission_Command& cmd);
static void do_wait_delay(const AP_Mission::Mission_Command& cmd);
static void do_within_distance(const AP_Mission::Mission_Command& cmd);
static void do_change_speed(const AP_Mission::Mission_Command& cmd);
static void do_set_home(const AP_Mission::Mission_Command& cmd);
static bool verify_nav_wp(const AP_Mission::Mission_Command& cmd);

...
```
Here you can find the definitions for some instance of [AP_Mission](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_Mission/AP_Mission.h#L43) class.

```cpp

static bool
start_command(const AP_Mission::Mission_Command& cmd)
{
    // log when new commands start
    if (should_log(MASK_LOG_CMD)) {
        Log_Write_Cmd(cmd);
    }

    // exit immediately if not in AUTO mode
    if (control_mode != AUTO) {
        return false;
    }
    ...
    ````
`start_commadn`funtion enables command use.If `MASK_LOG_CMD ` is active then logs for commands.
Also checks if the mode is AUTO, if not exits.

https://github.com/BeaglePilot/ardupilot/blob/master/APMrover2/commands_logic.pde#L27
