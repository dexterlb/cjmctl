#pragma once
#include <stdint.h>

typedef struct {
	float vel_coast; // default and maximum coast velocity (can be reduced dynamically)
	float vel_min;

	float pos_gain;

	float    target_reached_window;  // use 0 to never assume target is reached
	uint32_t target_reached_time_us; // timestamp must be small enough so that we don't leave the window
	                                 // while moving at vel_min for the given time
} control_position_cfg_t;

typedef struct {
	control_position_cfg_t* cfg;

	// state
	float    vel_coast;
	float    pos_err;
	uint32_t now_us;
	uint32_t target_reached_timestamp;
	float    pos_start;

	// params
	float pos_target;

	// inputs
	float pos_measured;

	// outputs
	float vel_output;
	bool  target_reached;
} control_position_t;

void control_position_init(control_position_t* cpos, control_position_cfg_t* cfg);
void control_position_update(control_position_t* cpos, uint32_t now_us);
void control_position_report_pos(control_position_t* cpos, float pos);
void control_position_target_pos(control_position_t* cpos, float pos);
void control_position_set_coast_vel(control_position_t* cpos, float vel);
