#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
	float vel_coast; // default and maximum coast velocity (can be reduced dynamically)
	float vel_min;

	float pos_gain;
	float acceleration;

	float    target_reached_window; // use 0 to never assume target is reached
	uint32_t changed_direction_wait_us;
	uint32_t target_reached_time_us; // timestamp must be small enough so that we don't leave the window
	                                 // while moving at vel_min for the given time
} control_position_cfg_t;

typedef struct {
	const control_position_cfg_t* cfg;

	// state
	float    vel_coast;
	float    pos_err;
	uint32_t now_us;
	uint32_t target_reached_timestamp;
	uint32_t changed_direction_wait_timestamp; // timestamp of wait start. See: control_position_cfg_t.changed_direction_wait
	float    vel_output_unshifted;
	float    prop_out;

	bool  paused;
	bool  unpause_requested;
	bool  ptru_requested;
	float ptru_vel;
	bool  direction_changed;

	// params
	float pos_target;

	// inputs
	float pos_measured;

	// outputs
	float vel_output;
	bool  target_reached;
} control_position_t;

void control_position_init(control_position_t* cpos, const control_position_cfg_t* cfg);
void control_position_update(control_position_t* cpos, uint32_t now_us);
void control_position_report_pos(control_position_t* cpos, float pos);
void control_position_target_pos(control_position_t* cpos, float pos);
void control_position_set_coast_vel(control_position_t* cpos, float vel);

void control_position_pause_if(control_position_t* cpos, bool pause);
void control_position_pause(control_position_t* cpos);
void control_position_unpause(control_position_t* cpos);
void control_position_stop(control_position_t* cpos);
void control_position_vel_ptru_mode(control_position_t* cpos, float vel);

float control_position_stop_pos(const control_position_t* cpos);

#ifdef __cplusplus
}
#include "control_position.hpp"
#endif
