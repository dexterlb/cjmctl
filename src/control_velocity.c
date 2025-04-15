#include "control_velocity.h"
#include "math_utils.h"

void control_velocity_init(control_velocity_t* cvel, control_velocity_cfg_t* cfg) {
    cvel->cfg = cfg;
    cvel->vel_target = 0;
    cvel->vel_target_preramp = 0;
    cvel->vel_measured = 0;
    cvel->vel_err_integral = 0;
    cvel->torque_output = 0;
    cvel->is_stopped = true;
    cvel->now_us = 0;
}

void control_velocity_update(control_velocity_t* cvel, uint32_t now_us) {
    // float dt = now - cvel->now;
    // cvel->now = now;

    float dt = calc_dt_from_timestamps_us(cvel->now_us, now_us);
    cvel->now_us = now_us;
    if (dt <= 0) {
        return;
    }

    if (!isnormal(dt)) {
        return;
    }

    // determine velocity target
    float vel_min = maxf(cvel->cfg->vel_min, 0.00001);
    if (fabs(cvel->vel_target_preramp) < vel_min) {
        // requested speed is too low, just stop
        cvel->vel_target = 0;
    } else if (fabs(cvel->vel_target) < vel_min) {
        // do not ramp below the min speed
        cvel->vel_target = vel_min * signf(cvel->vel_target_preramp);
    } else {
        // ramp the target velocity towards the preramp target
        cvel->vel_target = linear_ramp_to(
            cvel->vel_target,
            cvel->ramp_speed * dt,
            cvel->vel_target_preramp
        );
    }

    // see if we should get out of a stopped state
    if (minf(fabs(cvel->vel_target), fabs(cvel->vel_target_preramp)) >= vel_min) {
        cvel->is_stopped = false;
        cvel->rest_timer = now_us;
        cvel->rest_integral = 0;
    }

    // a very overengineered solution to ramping down torque when stopping
    // since torque goes up and down a lot while holding position, it must
    // be averaged before we begin ramping it down
    if (!cvel->is_stopped && now_us - cvel->rest_timer > cvel->cfg->rest_timeout) {
        cvel->torque_output = cvel->rest_integral / (cvel->rest_timer * 0.3);
        cvel->is_stopped = true;
    } else if (!cvel->is_stopped && now_us - cvel->rest_timer > cvel->cfg->rest_timeout * 0.7) {
        cvel->rest_integral += cvel->torque_output * dt;
    }

    // go limp if we're stopped
    if (cvel->is_stopped) {
        cvel->vel_target = 0;
        cvel->vel_err_integral = 0;
        cvel->torque_output = linear_ramp_to(
            cvel->torque_output,
            cvel->cfg->torque_rampdown_speed * dt,
            0
        );
        return;
    }

    // actual velocity control begins here
    cvel->vel_err = cvel->vel_target - cvel->vel_measured;

    // proportional component
    float prop_torque = cvel->cfg->vel_gain * cvel->vel_err;

    // integral component
    cvel->vel_err_integral += dt * cvel->vel_err;
    cvel->vel_err_integral = clampf(
        cvel->vel_err_integral,
        -cvel->cfg->vel_integrator_limit, cvel->cfg->vel_integrator_limit
    );
    float int_torque = cvel->cfg->vel_integrator_gain * cvel->vel_err_integral;

    // compute preliminary result
    float out = prop_torque + int_torque;

    // determine max torque
    float torque_max = cvel->cfg->torque_max;

    // don't try (too hard) to spin in the opposite direction to slow down
    if (out * cvel->vel_measured < 0) {
        torque_max = cvel->cfg->torque_opposite_max;
    }

    out = clampf(out, -torque_max, torque_max);

    // we're done
    cvel->torque_output = out;
}

void control_velocity_report_vel(control_velocity_t* cvel, float vel) {
    cvel->vel_measured = vel;
}

void control_velocity_target_vel(control_velocity_t* cvel, float vel, float ramp_speed) {
    cvel->vel_target_preramp = vel;
    cvel->ramp_speed = ramp_speed;
}
