# Steering.pde

The [Steering.pde](https://github.com/BeaglePilot/ardupilot/blob/master/APMrover2/Steering.pde) file includes the implementation of functions for triggering the rover, controlling the car movement (throttle,acceleration, servos...).

```cpp
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*****************************************
* Throttle slew limit
*****************************************/
static void throttle_slew_limit(int16_t last_throttle)
{
    // if slew limit rate is set to zero then do not slew limit
    if (g.throttle_slewrate) {
        // limit throttle change by the given percentage per second
        float temp = g.throttle_slewrate * G_Dt * 0.01f * fabsf(channel_throttle->radio_max - channel_throttle->radio_min);
        // allow a minimum change of 1 PWM per cycle
        if (temp < 1) {
            temp = 1;
        }
        channel_throttle->radio_out = constrain_int16(channel_throttle->radio_out, last_throttle - temp, last_throttle + temp);
    }
}
...
```
This function stores in `temp`float varible the value of the slew limit per cicle. If this value is smaller than 1 then it is updated to 1PWM, else it mantains ots value.

```cpp

/*
  check for triggering of start of auto mode
 */
static bool auto_check_trigger(void)
{
    // only applies to AUTO mode
    if (control_mode != AUTO) {
        return true;
    }

    // check for user pressing the auto trigger to off
    if (auto_triggered && g.auto_trigger_pin != -1 && check_digital_pin(g.auto_trigger_pin) == 1) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("AUTO triggered off"));
        auto_triggered = false;
        return false;
    }

    // if already triggered, then return true, so you don't
    // need to hold the switch down
    if (auto_triggered) {
        return true;
    }

    if (g.auto_trigger_pin == -1 && g.auto_kickstart == 0.0f) {
        // no trigger configured - let's go!
        auto_triggered = true;
        return true;
    }

    if (g.auto_trigger_pin != -1 && check_digital_pin(g.auto_trigger_pin) == 0) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("Triggered AUTO with pin"));
        auto_triggered = true;
        return true;
    }

    if (g.auto_kickstart != 0.0f) {
        float xaccel = ins.get_accel().x;
        if (xaccel >= g.auto_kickstart) {
            gcs_send_text_fmt(PSTR("Triggered AUTO xaccel=%.1f"), xaccel);
            auto_triggered = true;
            return true;
        }
    }

    return false;
}
...
```
This function first checks if the mode is Auto or not.After that checks for user pressing the auto trigger off.If it is off then send a text message (GCS) and updates ` auto_triggered` to false. If it is on , ` auto_triggered` is updated to true.Then checks if there is any trigger configured.After that checks if there is a pin for trigger. Last thing this function does is checking the `auto_kickstart` is not 0.0f.Then checks if the acceleration in x is greater than the `auto_kickstart` and sends a gcs message with this value, to end updates ` auto_triggered` to true.

```cpp
/*
  work out if we are going to use pivot steering
 */
static bool use_pivot_steering(void)
{
    if (control_mode >= AUTO && g.skid_steer_out && g.pivot_turn_angle != 0) {
        int16_t bearing_error = wrap_180_cd(nav_controller->target_bearing_cd() - ahrs.yaw_sensor) / 100;
        if (abs(bearing_error) > g.pivot_turn_angle) {
            return true;
        }
    }
    return false;
}
...
```
This slice of code implements a function to figure out if pivot steering is needed. Checks if `cotrol_mode`is set to AUTO and if the values of `skid_steer_out`and `pivot_turn_angle`are different from 0. If these conditions are true then `bearing_error`variable takes the `yaw_sensor`value divided by 100.When the absolute value of this `bearing_error`is greter than the `pivot_turn_angle`, then returns true, else returns false.

```cpp
/*
  calculate the throtte for auto-throttle modes
 */
static void calc_throttle(float target_speed)
{
    if (!auto_check_trigger()) {
        channel_throttle->servo_out = g.throttle_min.get();
        return;
    }

    float throttle_base = (fabsf(target_speed) / g.speed_cruise) * g.throttle_cruise;
    int throttle_target = throttle_base + throttle_nudge;

    /*
      reduce target speed in proportion to turning rate, up to the
      SPEED_TURN_GAIN percentage.
    */
    float steer_rate = fabsf(lateral_acceleration / (g.turn_max_g*GRAVITY_MSS));
    steer_rate = constrain_float(steer_rate, 0.0, 1.0);

    // use g.speed_turn_gain for a 90 degree turn, and in proportion
    // for other turn angles
    int32_t turn_angle = wrap_180_cd(next_navigation_leg_cd - ahrs.yaw_sensor);
    float speed_turn_ratio = constrain_float(fabsf(turn_angle / 9000.0f), 0, 1);
    float speed_turn_reduction = (100 - g.speed_turn_gain) * speed_turn_ratio * 0.01f;

    float reduction = 1.0 - steer_rate*speed_turn_reduction;

    if (control_mode >= AUTO && wp_distance <= g.speed_turn_dist) {
        // in auto-modes we reduce speed when approaching waypoints
        float reduction2 = 1.0 - speed_turn_reduction;
        if (reduction2 < reduction) {
            reduction = reduction2;
        }
    }

    // reduce the target speed by the reduction factor
    target_speed *= reduction;

    groundspeed_error = fabsf(target_speed) - ground_speed;

    throttle = throttle_target + (g.pidSpeedThrottle.get_pid(groundspeed_error * 100) / 100);

    // also reduce the throttle by the reduction factor. This gives a
    // much faster response in turns
    throttle *= reduction;

    if (in_reverse) {
        channel_throttle->servo_out = constrain_int16(-throttle, -g.throttle_max, -g.throttle_min);
    } else {
        channel_throttle->servo_out = constrain_int16(throttle, g.throttle_min, g.throttle_max);
    }

    if (!in_reverse && g.braking_percent != 0 && groundspeed_error < -g.braking_speederr) {
        // the user has asked to use reverse throttle to brake. Apply
        // it in proportion to the ground speed error, but only when
        // our ground speed error is more than BRAKING_SPEEDERR.
        //
        // We use a linear gain, with 0 gain at a ground speed error
        // of braking_speederr, and 100% gain when groundspeed_error
        // is 2*braking_speederr
        float brake_gain = constrain_float(((-groundspeed_error)-g.braking_speederr)/g.braking_speederr, 0, 1);
        int16_t braking_throttle = g.throttle_max * (g.braking_percent * 0.01f) * brake_gain;
        channel_throttle->servo_out = constrain_int16(-braking_throttle, -g.throttle_max, -g.throttle_min);

        // temporarily set us in reverse to allow the PWM setting to
        // go negative
        set_reverse(true);
    }

    if (use_pivot_steering()) {
        channel_throttle->servo_out = 0;
    }
}
...
```
This function `calc_throttle` calculates  the throtte for auto-throttle modes.First if `auto_check_trigger()`is not enabled then get the minimun value for throttle. Then `throttle_base`and `throttle_target` varibles are updated to throttle medium values.Also calculates the turning rate and stores it in `steer_rate`.Then defines ` turn_angle` ,` speed_turn_ratio`and  ` speed_turn_reduction`for controlling the angle, ratio and speed when turning.
When the mode is AUTO and the drone is aproaching to the turn point the `reduction`is aplied to its `target_speed`.This also changes the values of `groundspeed_error`and throttle, which are reduced by the reduction factor.If `in_reverse`is detected means that the user has asked to use reverse throttle to brake.Then, aplplies it in proportion to the ground speed error, but only when our ground speed error is more than `braking_speeder`.

https://github.com/BeaglePilot/ardupilot/blob/master/APMrover2/Steering.pde#L158
