#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    // General configuration
    float torque_max;
    float torque_opposite_max;      // use 0 if you don't want to apply opposite torque to brake
    float vel_min;

    // PI controller parameters
    float vel_gain;
    float vel_integrator_gain;
    float vel_integrator_limit;

    // Configuration for "rest mode" (going limp when velocity has been 0 for some time)
    float rest_timeout;             // use +inf if you want to always actively hold position
                                    // otherwise the controller will go limp after velocity
                                    // has been 0 for rest_timeout seconds

    float torque_rampdown_speed;    // torque units per second to ramp torque down with when going limp
} control_velocity_cfg_t;

typedef struct {
    control_velocity_cfg_t* cfg;

    // state
    float vel_err_integral;
    float vel_err;
    float rest_timer;
    float rest_integral;
    uint32_t now_us;

    // params
    float vel_target;
    float vel_target_preramp;
    float ramp_speed;

    // inputs
    float vel_measured;

    // outputs
    float torque_output;
    bool is_stopped;
} control_velocity_t;

// Usage:
// Call control_velocity_init() at startup.
// In your control loop, call control_velocity_report_vel() and then control_velocity_update()
// After that you can read the torque output and other outputs from the control_velocity_t
// structure and use them to control the motor.
// Call control_velocity_target_vel() when you want to change velocity
// If you don't want to ramp, use INFINITY as ramp_speed.

void control_velocity_init(control_velocity_t* cvel, control_velocity_cfg_t* cfg);
void control_velocity_update(control_velocity_t* cvel, uint32_t now_us);
void control_velocity_report_vel(control_velocity_t* cvel, float vel);
void control_velocity_target_vel(control_velocity_t* cvel, float vel, float ramp_speed);
