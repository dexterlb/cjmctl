#include <stdbool.h>
#include "control_position.h"
#include "math_utils.h"

void control_position_check_target_reached(control_position_t* cpos);

void control_position_init(control_position_t* cpos, control_position_cfg_t* cfg) {
	cpos->cfg                      = cfg;
	cpos->pos_measured             = 0;
	cpos->pos_target               = 0;
	cpos->pos_err                  = 0;
	cpos->now_us                   = 0;
	cpos->vel_output_unshifted     = 0;
	cpos->target_reached           = true;
	cpos->target_reached_timestamp = 0;
	cpos->vel_coast                = cfg->vel_coast;
	cpos->vel_output               = 0;
	cpos->paused = false;
	cpos->unpause_requested = false;
}

void control_position_pause_if(control_position_t* cpos, bool pause) {
	if (pause) {
		control_position_pause(cpos);
	} else {
		control_position_unpause(cpos);
	}
}

void control_position_pause(control_position_t* cpos) {
	cpos->paused = true;
}

void control_position_unpause(control_position_t* cpos) {
	if (cpos->paused) {
		cpos->unpause_requested = true;
	}
}

void control_position_update(control_position_t* cpos, uint32_t now_us) {
	float dt     = calc_dt_from_timestamps_us(cpos->now_us, now_us);
	cpos->now_us = now_us;

	if (cpos->unpause_requested) {
		cpos->unpause_requested = false;
		cpos->paused = false;
		return;
	}

	if (cpos->paused) {
		return;
	}

	if (dt <= 0) {
		return;
	}

	if (!isnormal(dt)) {
		return;
	}

	if (cpos->target_reached) {
		cpos->vel_output = 0;
		return;
	}

	cpos->pos_err = cpos->pos_target - cpos->pos_measured;

	cpos->prop_out = cpos->cfg->pos_gain * cpos->pos_err;

	float shifted_vel_coast = maxf(
		// it is important that we don't go over the max vel_coast, even if the user said so
		minf(cpos->vel_coast, cpos->cfg->vel_coast) - cpos->cfg->vel_min,
		0
	);

	float prop_sign = signf(cpos->prop_out);

	cpos->prop_out = clampf(
        cpos->prop_out,
        -shifted_vel_coast, shifted_vel_coast
    );

	if (fabs(cpos->prop_out) > fabs(cpos->vel_output_unshifted) || cpos->prop_out * cpos->vel_output_unshifted < 0) {
		cpos->vel_output_unshifted = linear_ramp_to(
			cpos->vel_output_unshifted,
			cpos->cfg->acceleration * dt,
			cpos->prop_out
		);
	} else {
		cpos->vel_output_unshifted = cpos->prop_out;
	}

	control_position_check_target_reached(cpos);
	cpos->vel_output = cpos->vel_output_unshifted + cpos->cfg->vel_min * prop_sign;
}

void control_position_report_pos(control_position_t* cpos, float pos) {
	cpos->pos_measured = pos;
}

void control_position_set_coast_vel(control_position_t* cpos, float vel) {
	cpos->vel_coast = vel;
}

void control_position_target_pos(control_position_t* cpos, float pos) {
	cpos->pos_target = pos;
	cpos->target_reached = false;
}

void control_position_check_target_reached(control_position_t* cpos) {
	float window = cpos->cfg->target_reached_window;
	if (window == 0.0f) {
		// user requested to never testify that target is reached
		return;
	}

	if (fabs(cpos->pos_measured - cpos->pos_target) >= window) {
		// we aren't close enough to the target
		cpos->target_reached_timestamp = cpos->now_us;
		return;
	}

	if (cpos->now_us - cpos->target_reached_timestamp <= cpos->cfg->target_reached_time_us) {
		// we haven't yet spent enough time in the "reached" window
		return;
	}

	cpos->target_reached = true;
}
